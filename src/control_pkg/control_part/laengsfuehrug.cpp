#include <iostream>
#include <cmath>
#include <iomanip>
#include <algorithm>

//????????????
struct ModelParams
{
	double tau;  //????? s /?????? 
	double Kv;   //?????????????? m/s
	double dt;   //????????
};
//??????????
struct PIParams
{
	double Kp;     //????????
	double Ki;     //????????
	double u_min;  //??С????
	double u_max;  //???????
};
//??????????  v_(k+1)=a*v_k+b*u_k
struct ModelState
{
	double v;  //??????
	double a;  //?????? a=e^(-dt/tau)
	double b;  //?????? b=Kv(1-a)
};
//PI???????????
struct PIState
{
	double u;      //?????????? ????u_k
	double e_pre;  //????????e_(k-1)
};
//?????
ModelState init_model(const ModelParams& p) 
{
	ModelState s;
	s.v = 0.0;
	s.a = std::exp(-p.dt / p.tau);
	s.b = p.Kv * (1.0 - s.a);
	return s;
}

PIState init_pi()
{
	PIState s;
	s.u = 0.0;
	s.e_pre = 0.0;
	return s;
}
//
double clamp(double x, double val_min, double val_max)
{
	if (x < val_min) return val_min;
	if (x > val_max) return val_max;
	return x;
}

double pi_step(const PIParams& p, PIState& s, double v_ref, double v_k, double dt)  //v_ref???????v_k???????dt????????
{
	double e_k = v_ref - v_k;
	double delta_u = p.Kp * (e_k - s.e_pre) + p.Ki * dt * e_k;
	s.u += delta_u;
	s.u = clamp(s.u, p.u_min, p.u_max);

	s.e_pre = e_k;

	return s.u;
}

double model_step(ModelState& s, double u)
{
	s.v = s.a * s.v + s.b * u;
	return s.v;
}

int main() {

	ModelParams model_params;
	model_params.tau = 1.0;
	model_params.Kv = 1.5;
	model_params.dt = 0.02;

	ModelState model = init_model(model_params);

	PIParams pi_params;
	pi_params.Ki = 0.7;
	pi_params.Kp = 0.5;
	pi_params.u_min = 0.0;
	pi_params.u_max = 1.0;

	PIState pi = init_pi();

	//????
	double sim_time = 5.0;
	int steps = static_cast<int>(sim_time / model_params.dt);

	double v_ref = 0.5;

	std::cout << "t\tv_ref\tv\tu\n";

	for (int k = 0; k <= steps; ++k) {
		double t = k * model_params.dt;

		if (std::abs(t - 2.5) < 1e-9) {
			v_ref = 0.25;
		}
		//?????????model.v????????u
		double u = pi_step(pi_params, pi, v_ref, model.v, model_params.dt);
		//??????????u???????v
		double v = model_step(model, u);

		std::cout << t << "\t" << v_ref << "\t" << v << "\t" << u << "\n";
	}
	return 0;

}