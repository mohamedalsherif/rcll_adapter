uint32 	msgid 
#  The ID of the message that is currently being processed or
#  was processed last.

bool final 
#  True, if the last generation triggered by a ComputeMessage has
#  been completed, false if it is still running. Also check the
#  msgid field if this field applies to the correct message.