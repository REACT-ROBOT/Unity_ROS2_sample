# Aggregated contact record between an entity link and one other object,
# accumulated since the last reset (or explicit clear).
string entity        # entity whose link touched something
string link          # link name
string other         # the other party: another entity's name, or the scenery
                     # object's name (the built-in floor is "Plane")
uint32 count         # number of contact-begin events
float64 first_time   # simulation time [s] of the first contact
float64 last_time    # simulation time [s] of the most recent contact
float64 max_impulse  # largest contact impulse magnitude seen [N s]
