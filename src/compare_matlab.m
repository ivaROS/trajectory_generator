path(path,'~/Documents/simulator/');
startup_rvc

func = near_ident_dynamics_tracking_func(1,2,1,.01);

func(0,[0;0;0;0;0;.5],0,0,0)

func(0,[0;0;0;0;0;.5],0,[1;0],0)

func(0,[0;0;0;0;0;.5],0,[0;1],0)


func(0,[1;2;1;0;0;.5],0,[0;1],0)

func(0,[1;2;1;1;1;.5],0,[0;1],0)

func(0,[1;2;1;1;1;.5],[5;1],[0;1],0)
