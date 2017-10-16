ROV teleoperation without any kind of feedback (one way only) and using gamepad.
This control doesn't use the mavros interface. It use the mavlink_cpp module instead.
The messages are sent encapsulated in packets of type DataLinkFrame (dccomms library)
