/*
	SELECT task data: SELECT id_task, deadline, id_robot_in_charge, seq_number, start_time, end_time FROM task ORDER BY id_task;
	SELECT robot data: SELECT id_robot, remaining_battery, medium_velocity from robot order by id_robot;
*/

delete from task;
delete from robot;


INSERT INTO robot (id_robot,description,status,max_payload,remaining_battery,discharge_factor,battery_threshold,medium_velocity,waiting_time,id_depot,id_current_location) VALUES (0,'Robot 0','FR',2,27,0.02,10,0.25,2,0,0);
INSERT INTO robot (id_robot,description,status,max_payload,remaining_battery,discharge_factor,battery_threshold,medium_velocity,waiting_time,id_depot,id_current_location) VALUES (1,'Robot 1','FR',2,21,0.02,10,0.25,2,4,4);
INSERT INTO robot (id_robot,description,status,max_payload,remaining_battery,discharge_factor,battery_threshold,medium_velocity,waiting_time,id_depot,id_current_location) VALUES (2,'Robot 2','FR',5,43,0.05,10,0.25,5,2,2);
INSERT INTO robot (id_robot,description,status,max_payload,remaining_battery,discharge_factor,battery_threshold,medium_velocity,waiting_time,id_depot,id_current_location) VALUES (3,'Robot 3','FR',5,76,0.05,10,0.25,5,6,6);
INSERT INTO robot (id_robot,description,status,max_payload,remaining_battery,discharge_factor,battery_threshold,medium_velocity,waiting_time,id_depot,id_current_location) VALUES (4,'Robot 4','FR',10,63,0.1,10,0.25,10,3,3);


INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (0,'T0-R0','N',2,344,80,8,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (1,'T1-R0','N',2,589,114,8,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (2,'T2-R0','N',2,752,144,8,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (3,'T3-R0','N',2,829,178,8,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (4,'T0-R1','N',2,356,71,12,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (5,'T1-R1','N',2,433,200,12,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (6,'T2-R1','N',2,513,202,12,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (7,'T0-R2','N',5,355,84,9,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (8,'T1-R2','N',5,547,149,9,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (9,'T2-R2','N',5,656,181,9,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (10,'T0-R3','N',5,247,163,11,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (11,'T1-R3','N',5,789,35,11,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (12,'T1-R3','N',5,1307,38,11,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (13,'T0-R4','N',10,254,189,10,NULL,NULL,NULL,NULL);
INSERT INTO task (id_task,description,status,payload,deadline,id_pick_up_location,id_delivery_location,id_robot_in_charge,seq_number,start_time,end_time) VALUES (14,'T0-R4','N',10,521,128,10,NULL,NULL,NULL,NULL);
