/*
	SELECT task data: SELECT id_task, status, deadline, id_robot_in_charge, seq_number, start_time, end_time FROM task ORDER BY id_task;
	SELECT robot data: SELECT id_robot, remaining_battery, medium_velocity from robot order by id_robot;
*/


delete from task;


UPDATE robot SET status = 'FR', id_current_location = id_depot, remaining_battery = 27 WHERE id_robot = 0;
UPDATE robot SET status = 'FR', id_current_location = id_depot, remaining_battery = 21 WHERE id_robot = 1;
UPDATE robot SET status = 'FR', id_current_location = id_depot, remaining_battery = 43 WHERE id_robot = 2;
UPDATE robot SET status = 'FR', id_current_location = id_depot, remaining_battery = 76 WHERE id_robot = 3;
UPDATE robot SET status = 'FR', id_current_location = id_depot, remaining_battery = 63 WHERE id_robot = 4;


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
