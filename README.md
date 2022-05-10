# Algoritmos para navegación autónoma de un vehículo sin conductor. TMR - AutoModelCar.

Desarrollo de  algoritmos para navegación de vehículos autónomos usando el simulador Webots y la plataforma ROS.

## REQUERIMIENTOS

* Ubuntu 18.04
* ROS-Melodic
* Webots R2021b

## INSTALACIÓN 

* $ cd
* $ git clone https://github.com/Davidtrejo590/Rayo_McQueen.git
* $ cd Rayo_McQueen/catkin_ws
* $ catkin_make -j2 -l2

## PRUEBAS
* Navegación sin obstáculos: navigation_no_obstacles.launch para 
* Navegación con obstáculos estáticos: navigation_static_obstacles.launch
* Navegación con obstáculos en movimiento: navigation_dynamic_obstacles.launch
* Estacionamiento autónomo: parking.launch

## TÓPICOS DE CONTROL
* ``/speed`` (std\_msgs/Float64): Velocidad [km/h]
* ``/steering`` (std\_msgs/Float64): Ángulo de dirección [rad]

## PARA PROBAR 

* $ export WEBOTS_HOME=/usr/local/webots
* $ cd
* $ source Rayo_McQueen/catkin_ws/devel/setup.bash
* $ roslaunch bring_up 'nombre_launch_disponible'.launch


* Si todo se instaló y compiló correctamente se debe de visualizar un Webots como el siguiente:
<img src="https://github.com/Davidtrejo590/Rayo_McQueen/blob/master/Media/webots.png" alt="Star Gazer App" width="700"/>
<br><br>
* En cuanto esté listo el simulador se inician los nodos necesarios para cada prueba y comienza la simulación.

## CONTACTO
Luis David Torres Trejo<br>
luisdavidtorrestrejo@gmail.com<br>




