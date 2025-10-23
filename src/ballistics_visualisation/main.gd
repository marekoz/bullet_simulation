
extends Node3D

@onready var program_path = "./simulation"

@onready var gun_x: SpinBox = $UI/GunPosContainer/GridContainer2/MSpinBox
@onready var gun_y: SpinBox = $UI/GunPosContainer/GridContainer2/MSpinBox2
@onready var gun_z: SpinBox = $UI/GunPosContainer/GridContainer2/MSpinBox3

@onready var target_x: SpinBox = $UI/TargetPosContainer/GridContainer2/TSpinBox
@onready var target_y: SpinBox = $UI/TargetPosContainer/GridContainer2/TSpinBox2
@onready var target_z: SpinBox = $UI/TargetPosContainer/GridContainer2/TSpinBox3

@onready var bullet_speed: SpinBox = $UI/GridContainer/SpinBox4
@onready var step_length: SpinBox = $UI/GridContainer/SpinBox5
@onready var bullet_mass: SpinBox = $UI/GridContainer/SpinBox6
@onready var result_label: Label = $UI/ResultLabel
@onready var bullet_path: Path3D = $Node3D/BulletPath

@onready var target_node = $Node3D/Target
@onready var muzzle_node = $Node3D/Muzzle
@onready var muzzle_label: Label3D = $Node3D/Muzzle/Label3D
@onready var target_label: Label3D = $Node3D/Target/Label3D

@onready var camera: Camera3D = $Node3D/Camera3D
@onready var sim_cd_timer: Timer = $SimCDTimer

@onready var help_button = $UI/HelpButton
@onready var help_container = $UI/HelpContainer

@onready var ground = $Node3D/GroundMesh
@onready var ground2 = $Node3D/GroundMesh2


@onready var can_simulate: bool = true

signal muzzle_changed
signal target_changed

func _ready():
	muzzle_changed.connect(change_muzzle_text)
	target_changed.connect(change_target_text)
	target_node.position = Vector3(target_x.value, target_z.value, target_y.value)
	muzzle_node.position = Vector3(gun_x.value, gun_z.value, gun_y.value)
	target_changed.emit()



# Input handling for setting muzzle/target position with mouse
func _process(_delta):
	if Input.is_action_pressed("Left Click") and Input.is_action_pressed("Target Change"):
		var proj_mouse_pos = get_cursor_position()
		if proj_mouse_pos != null:
			set_target_pos(proj_mouse_pos)
	if Input.is_action_pressed("Left Click") and Input.is_action_pressed("Muzzle Change"):
		var proj_mouse_pos = get_cursor_position()
		if proj_mouse_pos != null:
			set_muzzle_pos(proj_mouse_pos)
	if Input.is_action_just_pressed("Calculate"):
		simulate_trajectory()


func set_target_pos(pos: Vector3):
	if pos.y < 0:
		pos.y = 0
	target_x.value = pos.x
	target_y.value = pos.z
	target_z.value = pos.y
	target_node.position = pos
	target_changed.emit()


func set_muzzle_pos(pos: Vector3):
	if pos.y < 0:
		pos.y = 0
	gun_x.value = pos.x
	gun_y.value = pos.z
	gun_z.value = pos.y
	muzzle_node.position = pos
	muzzle_changed.emit()


# Get 3D world position under mouse cursor by raycasting
const PROJECTION_RAY_LENGTH = 10000
func get_cursor_position():
	var space_state = get_world_3d().direct_space_state
	var mouse_position = get_viewport().get_mouse_position()
	var ray_origin = camera.project_ray_origin(mouse_position)
	var ray_end = ray_origin + camera.project_ray_normal(mouse_position) * PROJECTION_RAY_LENGTH
	var params = PhysicsRayQueryParameters3D.new()
	params.from = ray_origin
	params.to = ray_end 
	params.collision_mask = 1#0b00000000000000000001
	params.exclude = [self]
	var intersection: Dictionary = space_state.intersect_ray(params)
	if intersection and intersection.has("position"):
		return intersection.position


# Scroll wheel input to adjust muzzle/target height
const VERTICAL_INCREMENT = 0.25
func _input(event):
	if event is InputEventMouseButton:
		match event.button_index:
			MOUSE_BUTTON_WHEEL_UP: 
				if Input.is_action_pressed("Muzzle Change"): 
					set_muzzle_pos(muzzle_node.position + Vector3(0, VERTICAL_INCREMENT, 0))
				if Input.is_action_pressed("Target Change"):
					set_target_pos(target_node.position + Vector3(0, VERTICAL_INCREMENT, 0))
			MOUSE_BUTTON_WHEEL_DOWN: # decrease fly velocity
				if Input.is_action_pressed("Muzzle Change"): 
					set_muzzle_pos(muzzle_node.position + Vector3(0, -VERTICAL_INCREMENT, 0))
				if Input.is_action_pressed("Target Change"):
					set_target_pos(target_node.position + Vector3(0, -VERTICAL_INCREMENT, 0))


# Parse output from external program into array of Vector3 points
func parse_output(data) -> Array:
	var points = []
	var rows = data.split("\n")
	var reading_trajectory = false
	result_label.text = ""
	for row in rows:
		if !reading_trajectory:
			if row.begins_with("Trajectory:"):
				reading_trajectory = true
			else:
				result_label.text += row + "\n"
		if reading_trajectory:
			var values = row.split(",")
			if values.size() != 3:
				continue
			var x = float(values[0])
			var z = float(values[1])
			var y = float(values[2])
			points.append(Vector3(x, y, z))
	return points

# Recreate Curve3D from array of Vector3 points
func remake_curve(points: Array) -> Curve3D:
	var curve = Curve3D.new()
	for p in points:
		curve.add_point(p)
	return curve

# Simulate trajectory by calling external program and parsing its output
func simulate_trajectory() -> void:
	if not can_simulate:
		return
	can_simulate = false
	sim_cd_timer.start()

	# Get values from UI controls
	var gun_pos = Vector3(gun_x.value, gun_y.value, gun_z.value)
	var target_pos = Vector3(target_x.value, target_y.value, target_z.value)
	var speed = bullet_speed.value
	var step = step_length.value
	var mass = bullet_mass.value
	
	# Create parameter string for vrgroup program
	var params = "%f %f %f %f %f %f %f %f %f -t" % [
		gun_pos.x, gun_pos.y, gun_pos.z,
		target_pos.x, target_pos.y, target_pos.z,
		speed, mass, step
	]
	# Execute the simulation in a thread
	var output = []
	var args = params.split(" ")
	var thread = Thread.new()
	var exit_code = 0
	thread.start(func():
		exit_code = OS.execute(program_path, args, output, true)
	)
	thread.wait_to_finish()

	if exit_code != 0:
		result_label.text = "ERROR: simulation failed with exit code: " + str(exit_code)
		result_label.text += "\nOutput:\n" + "\n".join(output)
		return

	var points = parse_output(output[0])
	if points.size() == 0:
		print("ERROR: No trajectory points found in data file")
		return
	var curve = remake_curve(points)
	bullet_path.curve = curve
	$Node3D/CSGPolygon3D.visible = true # csg polygon with the curve as shape


func _on_button_calculate_pressed() -> void:
	simulate_trajectory()


func change_muzzle_text() -> void:
	muzzle_label.text = "Muzzle\n%.2f, %.2f, %.2f" % [muzzle_node.position.x, muzzle_node.position.z, muzzle_node.position.y]

func change_target_text() -> void:
	target_label.text = "Target\n%.2f, %.2f, %.2f" % [target_node.position.x, target_node.position.z, target_node.position.y]


func _on_m_spin_box_value_changed(value: float) -> void:
	muzzle_node.position.x = value
	target_changed.emit()


func _on_m_spin_box_2_value_changed(value: float) -> void:
	muzzle_node.position.z = value
	muzzle_changed.emit()


func _on_m_spin_box_3_value_changed(value: float) -> void:
	muzzle_node.position.y = value
	muzzle_changed.emit()


func _on_t_spin_box_value_changed(value: float) -> void:
	target_node.position.x = value
	target_changed.emit()


func _on_t_spin_box_2_value_changed(value: float) -> void:
	target_node.position.z = value
	target_changed.emit()


func _on_t_spin_box_3_value_changed(value: float) -> void:
	target_node.position.y = value
	target_changed.emit()


func _on_help_button_pressed() -> void:
	help_button.text = "Hide Help" if !help_container.visible else "Show Help"
	help_container.visible = !help_container.visible


func _on_ground_button_2_pressed() -> void:
	ground2.visible = true
	ground.visible = false


func _on_ground_button_pressed() -> void:
	ground.visible = true
	ground2.visible = false


func _on_sim_cd_timer_timeout() -> void:
	can_simulate = true
