import numpy as np
from scipy.optimize import root_scalar, minimize_scalar


def find_zero_crossings(f, t_arr, method='brentq', fallback=True, **kwargs):
    """
    Find zero crossings of a function f(t). If no zero crossings are found,
    return the point where |f(t)| is minimal.

    Parameters
    ----------
    f :
        Function f(t).
    t_arr :
        1D array of increasing t-values.
    method :
        Root finding method to use (default: 'brentq').
    fallback :
        If True, return t where |f(t)| is minimal if no zero crossing is found.
    **kwargs :
        Extra keyword arguments for root_scalar.

    Returns
    -------
    List of float: t-values where f(t) = 0, or [t_min] if no crossings and
    fallback is True.
    """
    # Evaluate f(t) for each t in t_arr
    f_arr = np.zeros_like(t_arr)
    for i in range(len(t_arr)):
        f_arr[i] = f(t_arr[i])

    # Detect sign changes
    sign_changes = np.sign(f_arr[1:]) != np.sign(f_arr[:-1])
    crossing_indices = np.where(sign_changes)[0]

    # Try to find actual roots in intervals where sign changes
    roots = []
    for i in crossing_indices:
        t_low, t_high = t_arr[i], t_arr[i + 1]
        try:
            # noinspection PyTypeChecker
            res = root_scalar(f, bracket=[t_low, t_high], method=method, **kwargs)
            if res.converged:
                roots.append(res.root)
        except (ValueError, RuntimeError):
            pass

    # If no crossings were found: fallback
    if not roots and fallback:
        i = np.argmin(np.abs(f_arr))
        t_guess = t_arr[i]

        # Limit the search to a small bracket around t_guess
        i_low = max(i - 1, 0)
        i_high = min(i + 1, len(t_arr) - 1)
        t_low, t_high = t_arr[i_low], t_arr[i_high]

        # noinspection PyTypeChecker
        res = minimize_scalar(lambda t: abs(f(t)), bounds=(t_low, t_high), method='bounded')
        if res.success:
            return [res.x]
        else:
            return [t_guess]

    return roots
