global target

send_file {Flash} [lindex $argv 4] [lindex $argv 3] 0
compare_file  {Flash} [lindex $argv 4] [lindex $argv 3] 0

FLASH::ScriptGPNMV 4
