import lgpio

try:
    # Patch several constants which changed incompatibly between lg 0.1.6.0
    # and 0.2.0.0
    lgpio.SET_PULL_NONE
except AttributeError:
    lgpio.SET_PULL_NONE = lgpio.SET_BIAS_DISABLE
    lgpio.SET_PULL_UP = lgpio.SET_BIAS_PULL_UP
    lgpio.SET_PULL_DOWN = lgpio.SET_BIAS_PULL_DOWN
