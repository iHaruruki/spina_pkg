---
config:
  layout: elk
---
flowchart LR
    angle_send_node(["/angle_send_node"]) --> angle_cmd["/angle_cmd"]
    angle_cmd --> arm_controller(["/arm_controller"])
    angle_cmd@{ shape: rect}
    arm_controller --Serial--> spina[["spina</br>(Hardware)"]]
