"""Classic multi-camera point-cloud fusion for the teleop rig.

Two metric depth sensors are fused in the robot *base* frame (millimetres):

  * Zivid  : statically mounted, extrinsic ``T_zivid2base`` known by calibration.
  * D405   : wrist-mounted (eye-in-hand), extrinsic to the flange ``T_d405_2flange``
             obtained once via :mod:`handeye_calibrate`; combined at runtime with the
             live flange pose ``T_flange2base`` (HDR35 ``/system_<side>/pose_states``).

Transform naming convention used throughout: ``T_b_from_a`` (a.k.a. ``T_a2b`` in the
rest of this repo) maps a point expressed in frame ``a`` into frame ``b``:

    p_b_hom = T_b_from_a @ p_a_hom
"""
