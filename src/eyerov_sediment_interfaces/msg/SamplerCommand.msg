# Commands sent from SedimentCollector controller to hardware driver
uint8 CMD_IDLE=0
uint8 CMD_DEPLOY_ARM=1
uint8 CMD_PENETRATE=2
uint8 CMD_CLOSE_SCOOP=3
uint8 CMD_RETRACT_ARM=4
uint8 CMD_STOW_SAMPLE=5
uint8 CMD_EMERGENCY_RETRACT=6
uint8 CMD_HALT=7

uint8 command
float32 target_value          # Target extension/penetration (cm) depending on command
