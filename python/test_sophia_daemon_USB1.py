import sophia_daemon as sd

daemon = sd.SophiaDaemon(port='/dev/ttyUSB1')
daemon.run()
