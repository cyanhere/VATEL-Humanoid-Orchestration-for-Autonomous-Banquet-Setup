import os

if os.environ.get("DEXPICK_CTRL_LOG"):          # actuator-target log for deployment (platepick/ctrl_log.py)
    from platepick.ctrl_log import install
    install(os.environ["DEXPICK_CTRL_LOG"])
