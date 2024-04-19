//
// Created by xiang on 2021/11/5.
//

#ifndef IMU_INTEGRATION_H
#define IMU_INTEGRATION_H

#include "eigen_types.h"
#include "imu.h"
#define MAX_INI_COUNT (400)
/**
 * 本程序演示单纯靠IMU的积分
 */
class IMUIntegration {
   public:
    IMUIntegration(){};
    void Init(std::deque<IMUPtr> & imu ,const double& gravity_const, const Vec3d& init_bg, const Vec3d& init_ba){
        Vec3d cur_acc, cur_gyr;
        if (b_first_frame_)
        {
            reset();
            
            b_first_frame_ = false;
            const auto &imu_acc = imu.front()->acce_;
            const auto &gyr_acc = imu.front()->gyro_;
            mean_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
            mean_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();
        }

        printf("IMU Initializing: %.1f %%", double(init_iter_num) / MAX_INI_COUNT * 100);

        for (const auto &item : imu)
        {
            const auto &imu_acc = item->acce_;
            const auto &gyr_acc = item->gyro_;
            cur_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
            cur_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();

            mean_acc      += (cur_acc - mean_acc) / init_iter_num;
            mean_gyr      += (cur_gyr - mean_gyr) / init_iter_num;

            init_iter_num ++;
            if (init_iter_num > MAX_INI_COUNT){
                break;
            }
        }

        gravity_ =  - mean_acc * gravity_const / mean_acc.norm(); bg_ = init_bg; ba_ = init_ba;};

        // 增加imu读数
    void AddIMU(std::deque<IMUPtr> & imu) {
        std::unique_lock<std::mutex> lk_imu(mtx_imu);
        if(imu.empty()){
            return;
        }
        IMUPtr imu_next = std::make_shared<IMU>();
        imu_next =  imu.front();
        while (timestamp_begin > imu_next->timestamp_)
        {
            imu.pop_front();
            imu_next = imu.front();
        }

        // double dt = imu_next->timestamp_ - timestamp_begin;

        while (imu_next->timestamp_ > timestamp_begin && (timestamp_tmp - timestamp_begin)< 0.1 )
        {
            double dt = imu_next->timestamp_ - timestamp_tmp;
            
            p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt + 0.5 * (R_ * (imu_next->acce_ - ba_)) * dt * dt;
            v_ = v_ + R_ * (imu_next->acce_ - ba_) * dt + gravity_ * dt;
            R_ = R_ * Sophus::SO3d::exp((imu_next->gyro_ - bg_) * dt);
            timestamp_tmp = imu_next->timestamp_;
            imu.pop_front();
            imu_next = imu.front();
        }

    };
    //reset
    void reset(){
        v_ = Vec3d::Zero();
        p_ = Vec3d::Zero();
        R_ = SO3(Mat3d::Identity());
        timestamp_tmp = 0.0;
        timestamp_begin = 0.0;
        b_first_frame_ = true;
        bg_ = Vec3d::Zero();
        ba_ = Vec3d::Zero();
        mean_gyr, mean_acc;
        gravity_ = Vec3d(0, 0, -9.8);  // 重力
        init_iter_num = 1;
    };

    SO3d GetR() const { return R_; };
    Vec3d GetV() const { return v_; };
    Vec3d GetP() const { return p_; };
    void SetP(const Vec3d & p){ p_ = p;};
    void SetV(const Vec3d & v){ v_ = v;};
    void SetR(const Mat3d & R){ R_ = SO3(R);};
    void SetTimestamp(const double & t){ timestamp_tmp = timestamp_begin = t;};
   private:
    // 累计量
    SO3d R_;
    Vec3d v_ = Vec3d::Zero();
    Vec3d p_ = Vec3d::Zero();
    //上一帧雷达时间
    double timestamp_tmp = 0.0;
    double timestamp_begin = 0.0;
    std::mutex mtx_imu;
    bool b_first_frame_ = true;
    size_t init_iter_num = 1;
    // 零偏，由外部设定
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d mean_gyr, mean_acc;
    Vec3d gravity_ = Vec3d(0, 0, -9.8);  // 重力
};

using IMUIntegrationPtr = std::shared_ptr<IMUIntegration>;

#endif
