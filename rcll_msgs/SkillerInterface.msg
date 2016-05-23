string[1024] skill_string
#  Currently executed skill string, at least the first 1023 bytes of it.
#  Must be properly null-terminated.

string[128] error
#  String describing the error. Can be set by a skill when it fails.

uint32 exclusive_controller
#  Instance serial of the exclusive controller of the skiller. If this 
#  does not carry your instance serial your exec messages will be ignored. 
#  Aquire control with
#  the AquireControlMessage. Make sure you release control before exiting.

uint32 msgid
#   The ID of the message that is currently being processed,
#  	or 0 if no message is being processed.

uint8 SKILL_STATUS_INACTIVE = 0 # No skill is running.
uint8 SKILL_STATUS_FINAL = 1   	# The skill string has been successfully processed.
uint8 SKILL_STATUS_RUNNING = 2	# The execution is still running.
uint8 SKILL_STATUS_FAILED = 3 	# The execution failed and cannot succeed anymore.

uint8 status
#  The status of the current skill execution.
