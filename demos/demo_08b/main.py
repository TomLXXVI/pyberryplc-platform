from nicegui import app, ui
from hmi import CNCHMI


if __name__ == '__main__':

    hmi = CNCHMI(app, ui)
    hmi.run()
