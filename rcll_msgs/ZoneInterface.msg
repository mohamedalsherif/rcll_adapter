uint8 STATE_UNKNOWN = 0 # We did not detect it yet, but can't say no
uint8 STATE_NO = 1 		# We search everthing, there might be no MPS in the zone (or the skill can't find it)
uint8 STATE_YES = 2 	# We found the MPS and in the corresponding Position 3D interface are the possition 
uint8 STATE_MAYBE = 3 	# We found the MPS at a far possition, but not from close 
uint8 search_state 		# The state of the search of the actual zone.
uint32 tag_id 			# The ID of the tag, if set to -1 this means no TAG is known, e.g. MPS just found maybe (with laser-lines).