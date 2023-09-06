#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<Windows.h>
#include<cmath>
using namespace std;

enum states // границы карты
{
	target_found = 5,
	right_border = 1,
	upper_border = 2,
	left_border  = 3,
	bottom_border =4
};

// параметры симуляции
const double time_step = 0.1;
const double L_width = 1000;
const double L_length = 1000;
const double L_height = 1000;
size_t number_of_dynamic_obstacles;

double get_rand_double() // получение случайного значения угла
{
	int the_integer_part_of_number = rand() % 3; // целая часть вещественного числа
	int fractional_part_of_number = rand() % 100; // дробная часть вещественного числа
	return the_integer_part_of_number + fractional_part_of_number / 100;
}
struct Target
{
	double velocity;
	double x;
	double y;
	double z;
	double heading;
	double angle_of_attack;
	const double border_range = 1;
	double time_to_change_direction;
	bool is_active = true;
	Target(const double& X, const double& Y, const double& Z, const double& V, const double& H, const double& A)
	{
		x = X; y = Y; velocity = V; heading = H; angle_of_attack = A; z = Z;
	}
	Target()
	{
		x = 0; y = 0; velocity = 0; heading = 0;  angle_of_attack = 0;
	}
	void update_coordinates() // расчёт координат
	{
		if (abs(x - 0) <= border_range // left border 
			|| abs(y - 0) <= border_range // down border
			|| abs(y - L_width) <= border_range // upper border
			|| abs(x - L_length) <= border_range // down border
			)
		{
			heading -= (3.14 + get_rand_double());
			angle_of_attack -= (3.14 + get_rand_double());
		}
		
		x += velocity * cos(heading) * time_step;
		y += velocity * sin(heading) * time_step;
		z += velocity * sin(angle_of_attack) * time_step;
	}
	void display_coordinates() // вывод координат цели
	{
		cout << "[X]: " << x << endl;
		cout << "[Y]: " << y << endl;
		cout << "[Z]: " << z << endl;
	}
	void change_direction() // изменение курса и угла тангажа случайным образом
	{
		heading -= get_rand_double();
		angle_of_attack -= get_rand_double();
		time_to_change_direction += rand() % 1 / 2; // получение значения времени следующего изменения направления движения цели 
	}
};
struct Target_1
{
	double distance;
	double heading;
	double angle_of_attack;
	double x, y,z;
	double velocity;
	bool is_visible = false;
	Target_1(const double& D, const double& H)
	{
		distance = D; heading = H;
	}
	Target_1()
	{
		distance = 0; heading = 0; x = 0; y = 0; z = 0; angle_of_attack = 0;
	}
	
	void initialize_target(const double& D, const double& H)
	{
		distance = D; heading = H;
	}
};
struct UAV
{
	double x, y, z;
	double velocity;
	double heading;
	double angle_of_attack = 0;
	const double active_range = 100;
	const double capture_range = 30;
	const double border_range = 1;
	const double obstacle_range = 5;
	bool target_is_captured = false;
	
	Target_1 target; // прогнозируемое поведение цели в следующий момент времени
	UAV()
	{
		x = 0; y = 0; z = 0; velocity = 0; heading = 0;
	}
	UAV(const double& X, const double& Y, const double& V, const double& H)
	{
		x = X; y = Y; velocity = V; heading = H; z = 0;
	}
	double calculate_distance(const double& X, const double& Y, const double& Z)
	{
		return abs(sqrt(pow((x - X), 2) + pow((y - Y), 2) + pow((z - Z), 2)));
	}
	double calculate_heading(const double& X, const double& Y) // вычисление азимута на цель
	{
		return atan((y - Y) / (x - X));
	}
	double calculate_angle_of_attack(const double& L, const double& Z) // вычисление угла атаки до цели?
	{
		return atan((z - Z) / L);
	}
	void share_information(const Target& Obj, UAV& G) // обмен информацией между БПЛА
	{
		G.target.x = Obj.x;
		G.target.y = Obj.y;
		G.target.z = Obj.z;
		G.target.velocity = Obj.velocity;
		G.target.is_visible = true;
	}
	void use_radar(const Target& Obj, UAV& D, UAV& F, const Target* Obst) // обследование пространства
	{
		if (	abs(x - 0) <= border_range // left border 
			 || abs(y - 0) <= border_range // down border
			 || abs(y - L_width) <= border_range // upper border
			 || abs(x - L_length) <= border_range // down border
		   ) 
		{
			heading -= (3.14 + get_rand_double()); 
			angle_of_attack -= (3.14 + get_rand_double());
			//cout << "\nBorder\n";
			//system("pause");
		}

		for (size_t i = 0; i < number_of_dynamic_obstacles; i++)
		{
			if (Obst[i].is_active == true)
			{
				if ((abs(x - Obst[i].x) <= obstacle_range) || (abs(y - Obst[i].y) <= obstacle_range) || (abs(z - Obst[i].z) <= obstacle_range))
				{
					double heading_1 = heading - get_rand_double();
					double heading_2 = heading + get_rand_double();
					double angle_of_attack_1 = angle_of_attack - get_rand_double();
					double angle_of_attack_2 = angle_of_attack + get_rand_double();

					double proposed_obstacle_x = Obst[i].velocity * cos(Obst[i].heading) * time_step;
					double proposed_obstacle_y = Obst[i].velocity * sin(Obst[i].heading) * time_step;
					double proposed_obstacle_z = Obst[i].velocity * sin(Obst[i].angle_of_attack) * time_step;

					double temporary_x = velocity * sin(heading_1) * time_step;
					double temporary_y = velocity * cos(heading_1) * time_step;
					double temporary_z = velocity * sin(angle_of_attack_1) * time_step;

					double distance_1 = sqrt((pow((temporary_x - proposed_obstacle_x), 2) + pow((temporary_y - proposed_obstacle_y), 2) + pow((temporary_z - proposed_obstacle_z), 2)));

					temporary_x = velocity * sin(heading_2) * time_step;
					temporary_y = velocity * cos(heading_2) * time_step;
					temporary_z = velocity * sin(angle_of_attack_2) * time_step;

					double distance_2 = sqrt((pow((temporary_x - proposed_obstacle_x), 2) + pow((temporary_y - proposed_obstacle_y), 2) + pow((temporary_z - proposed_obstacle_z), 2)));

					if (distance_1 < distance_2)
					{
						heading = heading_1;
						angle_of_attack = angle_of_attack_1;
					}
					else
					{
						heading = heading_2;
						angle_of_attack = angle_of_attack_2;
					}
				}
			}
		}

		if (( abs(x - Obj.x) <= active_range) && (abs(y - Obj.y) <= active_range) && abs(z- Obj.z) <= active_range)
		{
			target.distance = calculate_distance(Obj.x, Obj.y, Obj.z);
			target.heading = calculate_heading(Obj.x, Obj.y);
			target.velocity = Obj.velocity;

			if (target.is_visible == false) // если не была до этого обнаружена цель
			{
				target.is_visible = true;
				//cout << "Target located in [" << Obj.x << "; " << Obj.y << "; " << Obj.z <<"]\n";
				//system("pause");
			}

			// передать данные об обнаруженной цели другим БЛА
			share_information(Obj,D); 
			share_information(Obj,F);
		}

		// если цель в радиусе захвата
		if (abs(x - Obj.x) <= capture_range && abs(y - Obj.y) <= capture_range && abs(z- Obj.z) <= capture_range)
		{
			target.distance = calculate_distance(Obj.x, Obj.y, Obj.z); // вычислить дистанцию до цели
			if (target.distance <= capture_range)
			{
				target_is_captured = true; // установить показатель захвата цели в ненулевое положение
				target.heading = calculate_heading(Obj.x, Obj.y); // вычисление 1 угла до цели
				double temporaty_length = sqrt(pow((x - Obj.x), 2) + pow((y - Obj.y), 2));
				target.angle_of_attack = calculate_angle_of_attack(temporaty_length, Obj.z); // вычисление 2 угла до цели а че за углы каво
				velocity = target.velocity; // изменить скорость движения на скорость цели
			}
		}
	}

	void update_coordinates() // вычисление координат БЛА
	{
		if (target.is_visible == false) // пока цель не обнаружена
		{
			x += velocity * cos(heading) * time_step;
			y += velocity * sin(heading) * time_step;
			z += velocity * sin(angle_of_attack) * time_step;
		}
		else // если цель замечена хотя бы одним БЛА
		{
			heading = atan( (target.y - y)/ (target.x - x)); // выбор нового угла курса
			angle_of_attack = atan( (target.z - z)/ (sqrt( pow((target.x-x),2)+ pow((target.y - y),2)))); // выбор нового угла атаки
			x += velocity * cos(heading) * time_step;
			y += velocity * sin(heading) * time_step;
			z += velocity * sin(angle_of_attack) * time_step;
		}
	}
	void display_coordinates() // вывод координат БЛА на экран консоли
	{
		cout << "[X]: " << x << endl;
		cout << "[Y]: " << y << endl;
		cout << "[Z]: " << z << endl;
		cout << target.distance << endl;
	}
};

int main()
{
	Target Object(300, 50, 0, 0.1, 1.5708, 0);
	UAV First(6, 30, 5, 0);
	UAV Second(100, 50, 5, 0.5236);
	UAV Third(50, 100, 5, 1.0472);
	double current_time = 0; // счётчик времени
	srand(time(0));
	const double pursuiting_time = 10; // заданное время преследования цели
	double time_of_pursuit = 0; // прошедшее время преследования
	double time_to_change_direction_for_object = rand()%1 / 2; // время, когда цель изменяет своё направление движения
	number_of_dynamic_obstacles = rand() % 10 + 1;
	Target** Obstacles = new Target* [number_of_dynamic_obstacles];
	for (size_t i = 0; i < number_of_dynamic_obstacles; i++)
	{
		double velocity = rand()%20 + 1;
		double x = rand()%1000;
		double y = rand()%1000;
		double z = rand()%1000;
		double heading = get_rand_double();
		double angle_of_attack = get_rand_double();
		Obstacles[i] = new Target(x,y,z,velocity, heading, angle_of_attack);
	}
	while (true)
	{
		First.use_radar(Object, Second, Third, *Obstacles); // исследование пространства 1 БЛА
		Second.use_radar(Object, First, Third, *Obstacles); // исследование пространства 2 БЛА
		Third.use_radar(Object, First, Second, *Obstacles); // исследование пространства 3 БЛА

		if (Object.time_to_change_direction == current_time) // (int)?
		{
			Object.change_direction(); // псевдослучайное изменение направления движения цели
		}
		for (size_t i = 0; i < number_of_dynamic_obstacles; i++)
		{
			if (Obstacles[i]->time_to_change_direction == current_time)
			{
				Obstacles[i]->change_direction(); // псевдослучайное изменение направления движения препятствия
			}
		}

		// вывод информации на экран
		cout << "Time: " << current_time << endl;
		cout << "Time of pursuit: " << time_of_pursuit << endl << endl;
		cout << "[Target]\n";
		Object.display_coordinates(); cout << endl;

		First.update_coordinates();
		cout << "[I UAV]\n";
		First.display_coordinates(); cout << endl;

		Second.update_coordinates();
		cout << "[II UAV]\n";
		Second.display_coordinates(); cout << endl;

		Third.update_coordinates();
		cout << "[III UAV]\n";
		Third.display_coordinates(); cout << endl << endl;

		// условия выхода из симуляции
		if (First.target_is_captured == true && Second.target_is_captured == true && Third.target_is_captured == true)
		{
			if (time_of_pursuit == 0) // вывести в первый раз на экран время захвата
			{
				//cout << "target is captured at " << current_time << endl;
				//system("pause");
			}
			time_of_pursuit += 0.1; // увеличить счётчик времени преследования
		}
		if ((int)time_of_pursuit == (int)pursuiting_time) // если время преследования вышло, прервать симуляцию
		{
			cout << "end of simulation\n";
			break;
		}

		Object.update_coordinates();     // движение цели
		for (size_t i = 0; i < number_of_dynamic_obstacles; i++)
		{
			Obstacles[i]->update_coordinates(); // движение i-ого препятствия
		}

		current_time += 0.1; // изменение времени
		system("cls");
	}
	return 0;
}