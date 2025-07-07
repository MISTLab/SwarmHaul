
function init(){
	# Only avilable in ARGOS
	# debug.trajectory.enable(10000,0,0,255)
}

function step(){
	log("Pos(",pose.position.x,",",pose.position.y,") yaw: ",pose.orientation.yaw)
}

function reset(){
	
}

function destroy(){
	
}
