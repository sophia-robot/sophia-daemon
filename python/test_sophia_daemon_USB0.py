import sophia_daemon as sd

daemon = sd.SophiaDaemon(port='/dev/ttyUSB0')
daemon.run()
