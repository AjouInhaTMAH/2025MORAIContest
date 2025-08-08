def rosmsg_to_dict(msg):
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            result[slot] = rosmsg_to_dict(value)
        return result
    elif isinstance(msg, (list, tuple)):
        return [rosmsg_to_dict(v) for v in msg]
    else:
        return msg 