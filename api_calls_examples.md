# API calls examples

## Robot on

```sh
curl -v -X POST http://cheetah.local/api/robot/on
```

## Robot off

```sh
curl -v -X POST http://cheetah.local/api/robot/off
```

## Clear

```sh
curl -v -X POST http://cheetah.local/api/recovery/clear
```

## Move command (single)

### Motor 1

```sh
curl -v -X POST http://cheetah.local/api/command \
     -H "Content-Type: application/json" \
     -d '{"motor_id": 1, "speed": 100, "command": "go_to_position", "position": 90}'
```

### Motor 2

```sh
curl -v -X POST http://cheetah.local/api/command \
     -H "Content-Type: application/json" \
     -d '{"motor_id": 2, "speed": 75, "command": "go_to_position", "position": 45}'
```

### Motor 3

```sh
curl -v -X POST http://cheetah.local/api/command \
     -H "Content-Type: application/json" \
     -d '{"motor_id": 3, "speed": 100, "command": "go_to_position", "position": 45}'
```

## Move command (batch)

```sh
curl -v -X POST http://cheetah.local/api/command/batch \
     -H "Content-Type: application/json" \
     -d '{
  "batch": [
    {"motor_id": 1, "speed": 50, "command": "go_to_position", "position": 45},
    {"motor_id": 2, "speed": 75, "command": "go_to_position", "position": 90},
    {"motor_id": 3, "speed": 100, "command": "go_to_position", "position": -45}
  ]
}'
```

```sh
curl -v -X POST http://cheetah.local/api/command/batch \
     -H "Content-Type: application/json" \
     -d '{
  "batch": [
    {"motor_id": 1, "speed": 50, "command": "go_to_position", "position": 45},
    {"motor_id": 2, "speed": 75, "command": "go_to_position", "position": 90},
    {"motor_id": 3, "speed": 100, "command": "go_to_position", "position": 135},
    {"motor_id": 1, "speed": 50, "command": "go_to_position", "position": 0},
    {"motor_id": 2, "speed": 75, "command": "go_to_position", "position": 0},
    {"motor_id": 3, "speed": 100, "command": "go_to_position", "position": 0}
  ]
}'
```
