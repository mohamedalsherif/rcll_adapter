string[32] frame				
#  Reference coordinate frame for the data.

int32 visibility_history	
#  The visibilitiy history indicates the number of consecutive positive or negative
#  sightings. If the history is negative, there have been as many negative sightings
#  (object not visible) as the absolute value of the history. A positive value denotes
#  as many positive sightings. 0 shall only be used during the initialization of the
#  interface or if the visibility history is not updated.

float64[4] rotation
#  Rotation quaternion relative to reference frame, ordered as (x, y, z, w).

float64[3] translation
#  Translation vector from the reference frame's origin, ordered as (x, y, z).

float64[36] covariance
#  Row-major representation of the 6x6 covariance matrix.
#  The orientation parameters use a fixed-axis representation.
#  In order, the parameters are:
#  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)