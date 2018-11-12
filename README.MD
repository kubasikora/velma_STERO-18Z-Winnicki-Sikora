Zespol: 

	velma_STERO-18Z-Winnicki-Sikora

Czlonkowie:

	Konrad Winnicki
	Jakub Sikora
	
Termin laboratorium:

	Czwartek, 16-20


Uruchamianie node'ów z tfami obiektów symulacji
roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=cafe_table::link frame_id:=table0
roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=cafe_table_0::link frame_id:=table1
roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=coke_can::link frame_id:=target

