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

    struct Random //OYM: ��������й�,���������Ŀ���,����������ô���İ�
{
    // std::mt19937 rng;
    std::minstd_rand rng;//OYM: һ��С��random
    // std::ranlux24 rng;
    // std::knuth_b rng;
    // std::default_random_engine rng;

    inline double random() { return std::uniform_real_distribution<double>(0, 1)(rng); }//OYM: 0-1��random

    inline std::size_t random_index(std::size_t s) { return std::uniform_int_distribution<size_t>(0, s - 1)(rng); } //OYM: size_t��ʾһ��������

    std::normal_distribution<double> normal_distribution; //OYM: ��̬�ֲ�,����������ѧ��
    inline double random_gauss() { return normal_distribution(rng); }//OYM: ��˹�ֲ�,������ô������

    inline double random(double min, double max) { return random() * (max - min) + min; }//OYM: ��Χ�ֲ�

    template <class e> inline e& random_element(std::vector<e>& l) { return l[random_index(l.size())]; } //OYM: �������һ��Ԫ��

    template <class e> inline const e& random_element(const std::vector<e>& l) { return l[random_index(l.size())]; } //OYM: ͬ��

    XORShift64 _xorshift; //OYM: �������������
    inline size_t fast_random_index(size_t mod) { return _xorshift() % mod; } //OYM: �������һ������
    template <class T> inline const T& fast_random_element(const std::vector<T>& v) { return v[fast_random_index(v.size())]; }//OYM: �������һ��Ԫ��

    static const size_t random_buffer_size = 1024 * 1024 * 8; //OYM: randomBuffer����,2^23����

    const double* make_random_buffer() //OYM: �������������
    {
        static std::vector<double> buf;
        buf.resize(random_buffer_size);
        for(auto& r : buf)
            r = random();
        return buf.data();
    }
    const double* random_buffer;
    size_t random_buffer_index;
    inline double fast_random() //OYM: �������,�������randomBuffer���е���
    {
        double r = random_buffer[random_buffer_index & (random_buffer_size - 1)];
        random_buffer_index++;
        return r;
    }

    const double* make_random_gauss_buffer()//OYM: ������˹�ֲ�
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
    inline double fast_random_gauss() //OYM: ���ٸ�˹���
    {
        double r = random_gauss_buffer[random_gauss_index & (random_buffer_size - 1)];
        random_gauss_index++;
        return r;
    }
    inline const double* fast_random_gauss_n(size_t n)//OYM: �����ӵĿ��ٸ�˹���
    {
        size_t i = random_gauss_index;
        random_gauss_index += n;
        if(random_gauss_index >= random_buffer_size) i = 0, random_gauss_index = n;
        return random_gauss_buffer + i;
    }

    Random(std::minstd_rand::result_type seed)
        : rng(seed)//OYM: ��������
    {
        random_buffer = make_random_buffer();
        random_buffer_index = _xorshift();
        random_gauss_buffer = make_random_gauss_buffer();
        random_gauss_index = _xorshift();
    }
};

struct IKBase : Random
{
    IKParams params; //OYM: ���в���,һ�����߰���Ķ���
    RobotFK model; //OYM: ģ��
    RobotInfo modelInfo; //OYM: ģ����Ϣ
    int thread_index;//OYM: �߳����
    Problem problem;//OYM: ��Ҫ��������
    std::vector<Frame> null_tip_frames; //OYM: ��ʼ֡?
    volatile int canceled; //OYM: ��ֹ����

    virtual void step() = 0; //OYM: һ���鷽��

    virtual const std::vector<double>& getSolution() const = 0; //OYM: �鷽��x2

    virtual void setParams(const IKParams& p) {} //OYM: �鷽��3

    IKBase(const IKParams& p) //OYM: ��ʼ��,˳����ʼ��һ���ϸϸ����Ķ���
        : Random(p.random_seed)
        , model(p.robot_model)
        , modelInfo(p.robot_model)
        , params(p)
    {
        setParams(p);
    }
    virtual ~IKBase() {} //OYM: ��������

    virtual void initialize(const Problem& problem)
    {
        this->problem = problem;//OYM: �趨����
        this->problem.initialize2();//OYM: ��ʼ������
        model.initialize(problem.tip_link_indices); //OYM: ģ�ͳ�ʼ��
        // active_variables = problem.active_variables;
        null_tip_frames.resize(problem.tip_link_indices.size()); //OYM: ��ʼλ�ó�ʼ��,�Ǹ�sizeӦ���ǹؽ�����
    }
    //�����Ҫ��Ӧ�ȵĻ��ֵ
    double computeSecondaryFitnessActiveVariables(const double* active_variable_positions) { return problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions); }
    //OYM: �����Ҫ��Ӧ�ȵ����е�ֵ
    double computeSecondaryFitnessAllVariables(const std::vector<double>& variable_positions) { return computeSecondaryFitnessActiveVariables(extractActiveVariables(variable_positions)); }
    //OYM: ������Ӧ�����е�ֵ
    double computeFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions); }
    //OYM: ���������
    double computeFitnessActiveVariables(const aligned_vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions); }
    //OYM: ������ҪĿ��ʹ�ҪĿ���������Ӧ��
    double computeCombinedFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        double ret = 0.0;
        ret += problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions);
        ret += problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions);
        return ret;
    }
    //OYM: ���������
    double computeCombinedFitnessActiveVariables(const aligned_vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        double ret = 0.0;
        ret += problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions);
        ret += problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions);
        return ret;
    }
    //OYM: ������л��ֵ
    bool checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.checkSolutionActiveVariables(tip_frames, active_variable_positions); }
    //OYM: ���������
    bool checkSolution(const std::vector<double>& variable_positions, const std::vector<Frame>& tips) { return checkSolutionActiveVariables(tips, extractActiveVariables(variable_positions)); }
    //OYM: ���岻��,������ʱ�洢�İ�
    std::vector<double> temp_active_variable_positions;
    //OYM: ��ȡ��ı���
    double* extractActiveVariables(const std::vector<double>& variable_positions)
    {
        temp_active_variable_positions.resize(problem.active_variables.size());
        for(size_t i = 0; i < temp_active_variable_positions.size(); i++)
            temp_active_variable_positions[i] = variable_positions[problem.active_variables[i]];
        return temp_active_variable_positions.data();
    }
    //OYM: ������Ӧ��
    double computeFitness(const std::vector<double>& variable_positions, const std::vector<Frame>& tip_frames) { return computeFitnessActiveVariables(tip_frames, extractActiveVariables(variable_positions)); }
    //OYM: ������Ӧ�ȵ�����
    double computeFitness(const std::vector<double>& variable_positions)
    {
        model.applyConfiguration(variable_positions);
        return computeFitness(variable_positions, model.getTipFrames());
    }
    //OYM: �շ���
    virtual size_t concurrency() const { return 1; }
};

typedef IKBase IKSolver;

typedef Factory<IKSolver, const IKParams&> IKFactory;
}
