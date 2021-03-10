
#Can Add functions as necessary

#Maps one variable within one range to another range, super useful Arduino function
def long_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#Constrains an input to be between an upper and lower bound
def constrain(val, min_val, max_val):
    return min(max(val, min_val), max_val)
