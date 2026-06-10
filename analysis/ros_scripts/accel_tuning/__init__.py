"""Feed-forward acceleration model tuning toolkit.

This subpackage provides modular routines for tuning the parameters of the
:class:`ateam_controls.RobotModel` that participate in the feed-forward
acceleration → wheel-current pipeline:

* Coulomb friction — separate constants per local axis:
  ``coulomb_friction_coefficient_{linear_x, linear_y, angular}``
* Viscous friction — separate constants per local axis:
  ``viscous_friction_coefficient_{linear_x, linear_y, angular}``
* Motor efficiency factor (``motor_efficiency_factor``)
* Rotational inertia (``iz``)

The friction-model relationship being tuned is, per local body axis::

    F_friction  = -c_visc * v_local - c_coul * sign(v_local)
    a_cmd_comp  = a_cmd - I^-1 * F_friction
    tau_wheel   = M^-1 * I * a_cmd_comp
    i_wheel     = tau_wheel / (Kt * eta)

so the commanded wheel current along a body axis is approximately::

    i_axis = (I * a_cmd + c_visc * v + c_coul) / (Kt * eta)

Tuning steps are intended to be run in this order, with each step using the
result of the previous one as its starting point:

1. ``coulomb x``     — find ``c_coul_linear_x``
2. ``coulomb y``     — find ``c_coul_linear_y`` (strafing has its own constants)
3. ``coulomb theta`` — find ``c_coul_angular``
4. ``viscous x|y|theta`` — search each axis's ``c_visc`` keeping the
   triangular profile linear (re-computing ``c_coul`` per candidate).
5. ``efficiency`` — search ``eta`` so realized accel matches commanded.
6. ``inertia`` — angular only; search ``iz`` so realized angular accel
   matches commanded under the tuned linear model.

Each routine can be run individually (``--mode single`` or ``--mode search``)
or composed by the :mod:`optimizer` module into a full end-to-end pass.
"""
