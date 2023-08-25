#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<Windows.h>
#include<cmath>
using namespace std;

enum states
{
	target_found = 5,
	right_border = 1,
	upper_border = 2,
	left_border  = 3,
	bottom_border =4
};

double time_step = 0.1;
double L_width = 1000;
double L_length = 1000;

//struct Friendly_UAV
//{
//	double x, y;
//	void update_coordinates(const double& X, const double& Y)
//	{
//		x = X; y = Y;
//	}
//};
struct Target
{
	double velocity;
	double x;
	double y;
	double z;
	double heading;
	double angle_of_attack;
	Target(const double& X, const double& Y, const double& Z, const double& V, const double& H)
	{
		x = X; y = Y; velocity = V; heading = H; angle_of_attack = 0; z = Z;
	}
	Target()
	{
		x = 0; y = 0; velocity = 0; heading = 0;  angle_of_attack = 0;
	}
	void update_coordinates()
	{
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
	double active_range = 100;
	double capture_range = 30;
	bool target_is_captured = false;

	//Friendly_UAV A;// change name
	//Friendly_UAV B; // change name
	Target_1 target;
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
	double calculate_heading(const double& X, const double& Y)
	{
		return atan((y - Y) / (x - X));
	}
	double calculate_angle_of_attack(const double& L, const double& Z)
	{
		return atan((z - Z) / L);
	}
	void share_information(const Target& Obj, UAV& G)
	{
		G.target.x = Obj.x;
		G.target.y = Obj.y;
		G.target.z = Obj.z;
		G.target.velocity = Obj.velocity;
	}
	void use_radar(const Target& Obj, UAV& D, UAV& F) // обследование пространства
	{
		if (( abs(x - Obj.x) <= active_range  /* && abs(x - Obj.x) > capture_range*/) && (abs(y - Obj.y) <= active_range /* && abs(y - Obj.y) > capture_range)*/))
		{
			target.distance = calculate_distance(Obj.x, Obj.y, Obj.z);
			target.heading = calculate_heading(Obj.x, Obj.y);
			target.velocity = Obj.velocity;
			if (target.is_visible == false)
			{
				target.is_visible = true;
				cout << "Target located in [" << Obj.x << "; " << Obj.y << "; " << Obj.z <<"]\n";
				system("pause");
			}
			D.target.is_visible = true; F.target.is_visible = true;
			share_information(Obj,D);
			share_information(Obj,F);
		}
		if (abs(x - Obj.x) <= capture_range && abs(y - Obj.y) <= capture_range)
		{
			target.distance = calculate_distance(Obj.x, Obj.y, Obj.z);
			target.heading = calculate_heading(Obj.x, Obj.y);
			velocity = target.velocity;
			target_is_captured = true;
		}
	}
	void update_coordinates()
	{
		if (target.is_visible == false) // пока цель не обнаружена
		{
			x += velocity * cos(heading) * time_step;
			y += velocity * sin(heading) * time_step;
			z += velocity * sin(angle_of_attack) * time_step;
		}
		else // если цель замечена хотя бы одним БПЛА
		{
			heading = atan( (target.y - y)/ (target.x - x)); // выбор нового угла курса
			angle_of_attack = atan( (target.z - z)/ (sqrt( pow((target.x-x),2)+ pow((target.y - y),2)))); // выбор нового угла атаки
			x += velocity * cos(heading) * time_step;
			y += velocity * sin(heading) * time_step;
			z += velocity * sin(angle_of_attack) * time_step;
		}
	}
	void display_coordinates() // вывод координат i-ого БПЛА на экран консоли
	{
		cout << "[X]: " << x << endl;
		cout << "[Y]: " << y << endl;
		cout << "[Z]: " << z << endl;
	}
};

int main()
{
	Target Object(300, 0, 0,0.1, 1.5708);
	UAV First(100, 0, 10, 0);
	UAV Second(100, 50, 10, 0.5236);
	UAV Third(10, 100, 10, 1.0472);
	double t = 0;
	while (true)
	{	
		First.use_radar(Object, Second, Third);
		Second.use_radar(Object, First, Third);
		Third.use_radar(Object, First, Second);
		Object.update_coordinates();
		cout << "[T]\n"; Object.display_coordinates(); cout << endl;
		First.update_coordinates();
		cout << "[I]\n"; First.display_coordinates(); cout << endl;
		Second.update_coordinates();
		cout << "[II]\n"; Second.display_coordinates(); cout << endl;
		Third.update_coordinates();
		cout << "[III]\n"; Third.display_coordinates();
		cout << endl << endl;
		if (First.target_is_captured == true && Second.target_is_captured == true && Third.target_is_captured == true)
		{
			cout << "end of simulation";
			break;
		}
		t += 0.1;
	}

}