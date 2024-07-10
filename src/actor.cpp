
#include "actor.h"
#define MAX_NAME_LEN 50
Actor::Actor()
{
    Initialize();
}
Actor::~Actor()
{
    cout << "Actor Memory delete" << endl;
}

void Actor::setup_weight(const char FC1_weight[], const char FC1_bias[], const char FC2_weight[], const char FC2_bias[], const char FC3_weight[], const char FC3_bias[], const char FC4_weight[], const char FC4_bias[])
{
    string line;
    vector<vector<float>> weight_w;
    vector<float> weight_b;

    ifstream file;
    file.open(FC1_weight);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        weight_w.push_back(row);
    }
    for (int r = 0; r < hidden_units; r++)
    {
        for (int c = 0; c < input_size; c++)
        {
            weight0[r][c] = weight_w[r][c];
            
        }
    }
    weight_w.clear();
    file.close();
    // ifstream file(FC1_bias);
    file.open(FC1_bias);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            weight_b.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
    }
    
    for (int r = 0; r < hidden_units; r++)
    {
        weight1[r] = weight_b[r];
        
    }
    weight_b.clear();
    file.close();

    file.open(FC2_weight);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        weight_w.push_back(row);
    }
    
    for (int r = 0; r < hidden_units; r++)
    {
        for (int c = 0; c < hidden_units; c++)
        {
            weight2[r][c] = weight_w[r][c];
        }
    }
    weight_w.clear();
    file.close();
    file.open(FC2_bias);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            weight_b.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
    }
    
    for (int r = 0; r < hidden_units; r++)
    {
        weight3[r] = weight_b[r];
        
    }
    weight_b.clear();
    file.close();
    file.open(FC3_weight);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        weight_w.push_back(row);
    }
    
    for (int r = 0; r < output_size; r++)
    {
        for (int c = 0; c < hidden_units; c++)
        {
            weight4[r][c] = weight_w[r][c];
        }
    }
    weight_w.clear();
    file.close();
    file.open(FC3_bias);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            weight_b.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
    }
    
    for (int r = 0; r < output_size; r++)
    {
        weight5[r] = weight_b[r];
        
    }
    weight_b.clear();
    file.close();

    

    // Fc1
    for (int i = 0; i < hidden_units; i++)
    {
        for (int j = 0; j < input_size; j++)
        {
            _FcW1(i, j) = weight0[i][j];
        }
    }
    for (int i = 0; i < hidden_units; i++)
    {
        _Fcb1(i) = weight1[i];
    }
    // Fc2
    for (int i = 0; i < hidden_units; i++)
    {
        for (int j = 0; j < hidden_units; j++)
        {
            _FcW2(i, j) = weight2[i][j];
        }
    }
    for (int i = 0; i < hidden_units; i++)
    {
        _Fcb2(i) = weight3[i];
    }

    // Fc3
    for (int i = 0; i < output_size; i++)
    {
        for (int j = 0; j < hidden_units; j++)
        {
            _FcW3(i, j) = weight4[i][j];
        }
    }
    for (int i = 0; i < output_size; i++)
    {
        _Fcb3(i) = weight5[i];
    }

    cout << "pass setup weights" << endl;
}
VectorXd Actor::Fc_layer1(VectorXd x)
{
    Fc1_output = (_FcW1 * x) + _Fcb1;
    output1 = V_ReLU(Fc1_output);
    return output1;
}
VectorXd Actor::Fc_layer2(VectorXd x)
{
    Fc2_output = (_FcW2 * x) + _Fcb2;
    output2 = V_ReLU(Fc2_output);

    return output2;
}
VectorXd Actor::Fc_layer3(VectorXd x)
{
    Fc3_output = (_FcW3 * x) + _Fcb3;
    // output3 = (Fc3_output);
    output3 = (Fc3_output).array().tanh();

    return output3;
}
VectorXd Actor::Forward(VectorXd x)
{
    FC1_out = Fc_layer1(x);
    FC2_out = Fc_layer2(FC1_out);
    FC3_out = Fc_layer3(FC2_out);
    return FC3_out;
}

void Actor::Initialize()
{
    input_size = 97; //91; // tau1,4 둘다 예측할 때
    output_size = 6;
    hidden_units = 256;

    x.setZero(input_size);

    _FcW1.setZero(hidden_units, input_size);
    _Fcb1.setZero(hidden_units);
    _FcW2.setZero(hidden_units, hidden_units);
    _Fcb2.setZero(hidden_units);
    _FcW3.setZero(output_size, hidden_units);
    _Fcb3.setZero(output_size);

    Fc1_output.setZero(hidden_units);
    output1.setZero(hidden_units);

    Fc2_output.setZero(hidden_units);
    output2.setZero(hidden_units);

    Fc3_output.setZero(output_size);
    output3.setZero(output_size);

    const char *FC1_weight = "/home/kist-robot2/Desktop/torch_cpp/weight/actor/w1.csv";
    const char *FC2_weight = "/home/kist-robot2/Desktop/torch_cpp/weight/actor/w2.csv";
    const char *FC3_weight = "/home/kist-robot2/Desktop/torch_cpp/weight/actor/w3.csv";
    const char *FC1_bias = "/home/kist-robot2/Desktop/torch_cpp/weight/actor/b1.csv";
    const char *FC2_bias = "/home/kist-robot2/Desktop/torch_cpp/weight/actor/b2.csv";
    const char *FC3_bias = "/home/kist-robot2/Desktop/torch_cpp/weight/actor/b3.csv";
    setup_weight(FC1_weight, FC1_bias, FC2_weight, FC2_bias, FC3_weight, FC3_bias);

}
VectorXd Actor::V_ReLU(VectorXd x)
{
    VectorXd _x, _relu_x;
    if (x.rows() > 2)
    {
        _relu_x.setZero(x.rows());
        _x.setZero(x.rows());
        _x = x;
        for (int i = 0; i < x.rows(); i++)
        {
            if (_x(i) < 0)
            {
                _relu_x(i) = 0;
            }
            else
            {
                _relu_x(i) = _x(i);
            }       
        }
    }
    else
    {
        _relu_x.setZero(x.rows());
        _x.setZero(x.rows());
        _x = x;
        for (int i = 0; i < x.rows(); i++)
        {
            if (_x(i) < 0)
            {
                _relu_x(i) = 0;
            }
            else
            {
                _relu_x(i) = _x(i);
            }
        }
    }
    return _relu_x;
}


int main(int argc, char **argv)
{
    // fisrt argument : test_obs.csv
    ifstream inputFile(argv[1]);
    vector<vector<float>> data;
    string line;
    while (getline(inputFile, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        data.push_back(row);
    }
    inputFile.close();

    // ifstream inputFile2(argv[2]);
    // vector<vector<float>> target;

    // while (getline(inputFile2, line))
    // {
    //     vector<float> row;
    //     istringstream iss(line);
    //     float value;
    //     while (iss >> value)
    //     {
    //         row.push_back(value);
    //         if (iss.peek() == ',')
    //             iss.ignore();
    //     }

    //     target.push_back(row);
    // }
    // inputFile2.close();
    
    Actor actor;
    // cout<<"actor last layer weight [3][3]"<<actor._FcW3[3][3]<<endl;
    // const char *FC1_weight = "../weight/FC1_weight.csv";
    // const char *FC2_weight = "../weight/FC2_weight.csv";
    // const char *FC3_weight = "../weight/FC3_weight.csv";
    // const char *FC1_bias = "../weight/FC1_bias.csv";
    // const char *FC2_bias = "../weight/FC2_bias.csv";
    // const char *FC3_bias = "../weight/FC3_bias.csv";
    // actor.setup_weight(FC1_weight, FC1_bias, FC2_weight, FC2_bias, FC3_weight, FC3_bias);

    std::chrono::steady_clock::time_point st_start_time;

    VectorXd input(97);
    VectorXd answer(6);
    for (size_t i = 0; i < 10; ++i)
    {
        for (size_t j = 0; j < data[i].size(); ++j)
        {
            input(j) = data[i][j];
        }
        // for (size_t j = 0; j < target[i].size(); ++j)
        // {
        //     answer(j) = target[i][j];
        // }
        st_start_time = std::chrono::steady_clock::now();

        double control_time_real_ = 0.0;
        
        auto output = actor.Forward(input);
        // cout<<input<<endl;
        control_time_real_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - st_start_time).count();
        control_time_real_ = control_time_real_ / 1000;
        cout << "all : " << control_time_real_ << "ms" << endl
             << endl;
        // cout << "data : " << input.transpose() << endl;
        // cout << answer << endl
        //      << endl;
        cout << output << endl;
    }
    return 0;
}