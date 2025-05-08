# Tipo de mensaje entre robots patrulla

std_msgs/Header header                  # Standard header para timestamps/frame_id
geometry_msgs/Vector3 victim_wp         # Victima waypoint [x, y, z]
string waypoint_id                      # Id para waypoint
bool victim_report                      # estado de reporte de victimas
bool robot_in_location                  # estado posicion robot respecto a victima

