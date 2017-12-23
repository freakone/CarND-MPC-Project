#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
    return M_PI;
}
double deg2rad(double x) {
    return x * pi() / 180;
}
double rad2deg(double x) {
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
    int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main() {
    uWS::Hub h;

    const int latency_ms = 100;

    // MPC is initialized here!
    MPC mpc;
    h.onMessage([ & mpc](uWS::WebSocket < uWS::SERVER > ws, char * data, size_t length,
        uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get < string > ();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector < double > ptsx = j[1]["ptsx"];
                    vector < double > ptsy = j[1]["ptsy"];
                    const double px = j[1]["x"];
                    const double py = j[1]["y"];
                    const double psi = j[1]["psi"];
                    const double v = j[1]["speed"];
                    const double throttle = j[1]["throttle"];
                    const double steering_angle =  j[1]["steering_angle"];

                    auto matrix = mpc.transformCoordinates(ptsx, ptsy, px, py, psi);
                    const auto & xv = matrix.row(0);
                    const auto & yv = matrix.row(1);

                    auto coeffs = polyfit(xv, yv, 3);
                    double cte = polyeval(coeffs, 0);
                    double epsi = -atan(coeffs[1]);

                    Eigen::VectorXd state(6);
                    //state << 0, 0, 0, v, cte, epsi;
                    const double latency_s = latency_ms/1000;
                    const double Lf = 2.67;
                    state(0) = v * latency_s;
                    state(1) = 0;
                    state(2) = -v * steering_angle / Lf * latency_s;
                    state(3) = v + throttle * latency_s;
                    state(4) = cte + v * sin(epsi) * latency_s;
                    state(5) = epsi + state(2); //predict the state for Latency compenstation

                    auto solve = mpc.Solve(state, coeffs);

                    json msgJson;
                    msgJson["steering_angle"] = -solve.angle;
                    msgJson["throttle"] = solve.throttle;
                    msgJson["mpc_x"] = solve.x_prediction;
                    msgJson["mpc_y"] = solve.y_prediction;
                    msgJson["next_x"] = mpc.toStdVector(xv);
                    msgJson["next_y"] = mpc.toStdVector(yv);

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(latency_ms));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char * data,
        size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([ & h](uWS::WebSocket < uWS::SERVER > ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([ & h](uWS::WebSocket < uWS::SERVER > ws, int code,
        char * message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}