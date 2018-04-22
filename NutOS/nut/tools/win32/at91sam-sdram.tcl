global target

# Init SDRAM
#SDRAM::initSDRAM_133
GENERIC::Init $RAM::appletAddr $RAM::appletFileName [list $::target(comType) $GENERIC::traceLevel $BOARD::vddmem $BOARD::ramType]

send_file {SDRAM} [lindex $argv 4] [lindex $argv 3] 0
compare_file {SDRAM} [lindex $argv 4] [lindex $argv 3] 0

# Start the uploaded image
go [lindex $argv 3]
