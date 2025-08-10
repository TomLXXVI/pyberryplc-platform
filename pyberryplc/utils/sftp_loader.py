from __future__ import annotations
from dataclasses import dataclass
import os
import socket
import tomllib
import paramiko
import posixpath
from typing import Iterable
import hashlib


@dataclass
class SFTPConfig:
    host: str
    port: int
    user: str
    dest_dir: str
    remote_motor_config_path: str
    auth: str = "key"  # "key" | "password"
    key_file: str | None = None
    key_passphrase: str | None = None
    password: str | None = None
    known_hosts: str | None = None
    connect_timeout: int = 10
    operation_timeout: int = 30
    atomic_put: bool = True
    mkdir_mode: int = 0o755

    @classmethod
    def from_file(cls, path: str | os.PathLike) -> "SFTPConfig":
        data = tomllib.load(open(path, "rb"))
        return cls(**data)


class SFTPUploader:

    def __init__(self, cfg: SFTPConfig):
        self.cfg = cfg

    @classmethod
    def from_file(cls, path: str | os.PathLike) -> "SFTPUploader":
        return cls(SFTPConfig.from_file(path))

    def _connect(self) -> tuple[paramiko.SSHClient, paramiko.SFTPClient]:
        client = paramiko.SSHClient()
        if self.cfg.known_hosts:
            client.load_host_keys(self.cfg.known_hosts)
        else:
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        connect_kwargs = {
            "hostname": self.cfg.host,
            "port": self.cfg.port,
            "username": self.cfg.user,
            "timeout": self.cfg.connect_timeout,
        }

        if self.cfg.auth == "key":
            if not self.cfg.key_file:
                raise ValueError("key auth selected but key_file is missing")
            # Try RSA or Ed25519
            try:
                pkey = paramiko.RSAKey.from_private_key_file(
                    self.cfg.key_file,
                    password=self.cfg.key_passphrase
                )
            except paramiko.PasswordRequiredException:
                raise
            except Exception:
                # fallback to Ed25519
                pkey = paramiko.Ed25519Key.from_private_key_file(
                    self.cfg.key_file,
                    password=self.cfg.key_passphrase
                )
            # noinspection PyTypeChecker
            connect_kwargs["pkey"] = pkey
        else:
            if not self.cfg.password:
                raise ValueError("password auth selected but password is missing")
            connect_kwargs["password"] = self.cfg.password

        client.connect(**connect_kwargs)
        sftp = client.open_sftp()
        sftp.get_channel().settimeout(self.cfg.operation_timeout)
        return client, sftp

    def _ensure_remote_dir(
        self,
        sftp: paramiko.SFTPClient,
        directory: str
    ) -> None:
        # build POSIX‑path recursively
        parts = [p for p in directory.strip("/").split("/") if p]
        current = "/"
        for part in parts:
            current = posixpath.join(current, part)
            try:
                sftp.stat(current)
            except FileNotFoundError:
                sftp.mkdir(current, mode=self.cfg.mkdir_mode)

    def _atomic_put(
        self,
        sftp: paramiko.SFTPClient,
        local_path: str,
        remote_path: str
    ) -> None:
        if not self.cfg.atomic_put:
            sftp.put(local_path, remote_path)
            return
        tmp_path = f"{remote_path}.tmp.{os.getpid()}"
        sftp.put(local_path, tmp_path)
        sftp.rename(tmp_path, remote_path)

    def upload_files(
        self,
        local_paths: Iterable[str | os.PathLike]
    ) -> list[str]:
        client = None
        sftp = None
        try:
            client, sftp = self._connect()
            self._ensure_remote_dir(sftp, self.cfg.dest_dir)
            remote_paths = []
            for lp in local_paths:
                fname = os.path.basename(str(lp))
                rp = posixpath.join(self.cfg.dest_dir, fname)
                self._atomic_put(sftp, str(lp), rp)
                remote_paths.append(rp)
            return remote_paths
        except (paramiko.SSHException, socket.timeout, OSError) as e:
            raise RuntimeError(f"SFTP upload failed: {e}") from e
        finally:
            try:
                if sftp: sftp.close()
            finally:
                if client: client.close()


@dataclass
class RemoteFileInfo:
    path: str
    sha256: str
    mtime: float


class SFTPDownloader:

    def __init__(self, cfg: SFTPConfig):
        self.cfg = cfg

    @classmethod
    def from_file(cls, path: str | os.PathLike) -> "SFTPDownloader":
        return cls(SFTPConfig.from_file(path))

    def _connect(self):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        kwargs = dict(
            hostname=self.cfg.host, port=self.cfg.port,
            username=self.cfg.user, timeout=self.cfg.connect_timeout
        )
        if self.cfg.auth == "key":
            try:
                pkey = paramiko.RSAKey.from_private_key_file(
                    self.cfg.key_file,
                    password=self.cfg.key_passphrase
                )
            except Exception:
                pkey = paramiko.Ed25519Key.from_private_key_file(
                    self.cfg.key_file,
                    password=self.cfg.key_passphrase
                )
            # noinspection PyTypeChecker
            kwargs["pkey"] = pkey
        else:
            kwargs["password"] = self.cfg.password
        client.connect(**kwargs)
        sftp = client.open_sftp()
        return client, sftp

    def download_file(self, remote_path: str, local_path: str) -> RemoteFileInfo:
        client = None
        sftp = None
        try:
            # Connect with the Pi
            client, sftp = self._connect()

            # Download file.
            sftp.get(remote_path, local_path)

            # Get remote stats
            st = sftp.stat(remote_path)

            # Calculate SHA256 of the downloaded file.
            h = hashlib.sha256()
            with open(local_path, "rb") as f:
                for chunk in iter(lambda: f.read(65536), b""):
                    h.update(chunk)

            return RemoteFileInfo(
                path=remote_path,
                sha256=h.hexdigest(),
                mtime=st.st_mtime
            )

        except FileNotFoundError:
            raise RuntimeError(f"Remote file not found: {remote_path}")
        except paramiko.SSHException as e:
            raise RuntimeError(f"SFTP SSH error: {e}") from e
        except OSError as e:
            raise RuntimeError(f"Local file error: {e}") from e
        finally:
            if sftp is not None:
                try:
                    sftp.close()
                except Exception:
                    pass
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
