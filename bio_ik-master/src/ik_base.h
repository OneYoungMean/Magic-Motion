/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <bio_ik/goal.h>

#include "forward_kinematics.h"
#include "problem.h"
#include "utils.h"
#include <bio_ik/robot_info.h>

#include <random>

namespace bio_ik
{

    struct Random //OYM: 跟随机数有关,看到就耐心看完,看看别人怎么做的吧
{
    // std::mt19937 rng;
    std::minstd_rand rng;//OYM: 一个小的random
    // std::ranlux24 rng;
    // std::knuth_b rng;
    // std::default_random_engine rng;

    inline double random() { return std::uniform_real_distribution<double>(0, 1)(rng); }//OYM: 0-1的random

    inline std::size_t random_index(std::size_t s) { return std::uniform_int_distribution<size_t>(0, s - 1)(rng); } //OYM: size_t表示一个正整数

    std::normal_distribution<double> normal_distribution; //OYM: 正态分布,疑似内置数学库
    inline double random_gauss() { return normal_distribution(rng); }//OYM: 高斯分布,他是怎么做到的

    inline double random(double min, double max) { return random() * (max - min) + min; }//OYM: 范围分布

    template <class e> inline e& random_element(std::vector<e>& l) { return l[random_index(l.size())]; } //OYM: 随机返回一个元素

    template <class e> inline const e& random_element(const std::vector<e>& l) { return l[random_index(l.size())]; } //OYM: 同上

    XORShift64 _xorshift; //OYM: 大正整数随机库
    inline size_t fast_random_index(size_t mod) { return _xorshift() % mod; } //OYM: 还是随机一个整数
    template <class T> inline const T& fast_random_element(const std::vector<T>& v) { return v[fast_random_index(v.size())]; }//OYM: 随机返回一个元素

    static const size_t random_buffer_size = 1024 * 1024 * 8; //OYM: randomBuffer缓冲,2^23个数

    const double* make_random_buffer() //OYM: 创建随机数缓冲
    {
        static std::vector<double> buf;
        buf.resize(random_buffer_size);
        for(auto& r : buf)
            r = random();
        return buf.data();
    }
    const double* random_buffer;
    size_t random_buffer_index;
    inline double fast_random() //OYM: 快速随机,随机访问randomBuffer当中的数
    {
        double r = random_buffer[random_buffer_index & (random_buffer_size - 1)];
        random_buffer_index++;
        return r;
    }

    const double* make_random_gauss_buffer()//OYM: 构建高斯分布
    {
        // LOG("make_random_gauss_buffer");
        static std::vector<double> buf;
        buf.resize(random_buffer_size);
        for(auto& r : buf)
            r = random_gauss();
        return buf.data();
    }
    const double* random_gauss_buffer;
    size_t random_gauss_index;
    inline double fast_random_gauss() //OYM: 快速高斯随机
    {
        double r = random_gauss_buffer[random_gauss_index & (random_buffer_size - 1)];
        random_gauss_index++;
        return r;
    }
    inline const double* fast_random_gauss_n(size_t n)//OYM: 带种子的快速高斯随机
    {
        size_t i = random_gauss_index;
        random_gauss_index += n;
        if(random_gauss_index >= random_buffer_size) i = 0, random_gauss_index = n;
        return random_gauss_buffer + i;
    }

    Random(std::minstd_rand::result_type seed)
        : rng(seed)//OYM: 设置种子
    {
        random_buffer = make_random_buffer();
        random_buffer_index = _xorshift();
        random_gauss_buffer = make_random_gauss_buffer();
        random_gauss_index = _xorshift();
    }
};

struct IKBase : Random
{
    IKParams params; //OYM: 运行参数,一堆乱七八糟的东西
    RobotFK model; //OYM: 模型
    RobotInfo modelInfo; //OYM: 模型信息
    int thread_index;//OYM: 线程序号
    Problem problem;//OYM: 需要求解的问题
    std::vector<Frame> null_tip_frames; //OYM: 初始帧?
    volatile int canceled; //OYM: 终止次数

    virtual void step() = 0; //OYM: 一个虚方法

    virtual const std::vector<double>& getSolution() const = 0; //OYM: 虚方法x2

    virtual void setParams(const IKParams& p) {} //OYM: 虚方法3

    IKBase(const IKParams& p) //OYM: 初始化,顺带初始化一大堆细细碎碎的东西
        : Random(p.random_seed)
        , model(p.robot_model)
        , modelInfo(p.robot_model)
        , params(p)
    {
        setParams(p);
    }
    virtual ~IKBase() {} //OYM: 析构函数

    virtual void initialize(const Problem& problem)
    {
        this->problem = problem;//OYM: 设定问题
        this->problem.initialize2();//OYM: 初始化问题
        model.initialize(problem.tip_link_indices); //OYM: 模型初始化
        // active_variables = problem.active_variables;
        null_tip_frames.resize(problem.tip_link_indices.size()); //OYM: 初始位置初始化,那个size应该是关节数量
    }
    //计算次要适应度的活动的值
    double computeSecondaryFitnessActiveVariables(const double* active_variable_positions) { return problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions); }
    //OYM: 计算次要适应度的所有的值
    double computeSecondaryFitnessAllVariables(const std::vector<double>& variable_positions) { return computeSecondaryFitnessActiveVariables(extractActiveVariables(variable_positions)); }
    //OYM: 计算适应度所有的值
    double computeFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions); }
    //OYM: 上面的重载
    double computeFitnessActiveVariables(const aligned_vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions); }
    //OYM: 计算首要目标和次要目标的联合适应度
    double computeCombinedFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        double ret = 0.0;
        ret += problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions);
        ret += problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions);
        return ret;
    }
    //OYM: 上面的重载
    double computeCombinedFitnessActiveVariables(const aligned_vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        double ret = 0.0;
        ret += problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions);
        ret += problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions);
        return ret;
    }
    //OYM: 检查所有活动的值
    bool checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.checkSolutionActiveVariables(tip_frames, active_variable_positions); }
    //OYM: 检查解决方案
    bool checkSolution(const std::vector<double>& variable_positions, const std::vector<Frame>& tips) { return checkSolutionActiveVariables(tips, extractActiveVariables(variable_positions)); }
    //OYM: 意义不明,用来临时存储的吧
    std::vector<double> temp_active_variable_positions;
    //OYM: 提取活动的变量
    double* extractActiveVariables(const std::vector<double>& variable_positions)
    {
        temp_active_variable_positions.resize(problem.active_variables.size());
        for(size_t i = 0; i < temp_active_variable_positions.size(); i++)
            temp_active_variable_positions[i] = variable_positions[problem.active_variables[i]];
        return temp_active_variable_positions.data();
    }
    //OYM: 计算适应度
    double computeFitness(const std::vector<double>& variable_positions, const std::vector<Frame>& tip_frames) { return computeFitnessActiveVariables(tip_frames, extractActiveVariables(variable_positions)); }
    //OYM: 计算适应度的重载
    double computeFitness(const std::vector<double>& variable_positions)
    {
        model.applyConfiguration(variable_positions);
        return computeFitness(variable_positions, model.getTipFrames());
    }
    //OYM: 空方法
    virtual size_t concurrency() const { return 1; }
};

typedef IKBase IKSolver;

typedef Factory<IKSolver, const IKParams&> IKFactory;
}
