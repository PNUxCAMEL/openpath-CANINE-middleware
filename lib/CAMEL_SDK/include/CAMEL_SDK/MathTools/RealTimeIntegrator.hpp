//
// Created by ys on 25. 3. 13.
//

#ifndef REALTIMEINTEGRATOR_HPP
#define REALTIMEINTEGRATOR_HPP

#include <Eigen/Core>

class RealTimeIntegrator {
public:
    RealTimeIntegrator(double dt, double window_duration)
        : dt_(dt),
          window_samples_(static_cast<size_t>(window_duration / dt)),
          buffer_(window_samples_, Eigen::Vector3d::Zero()),
          current_index_(0),
          integral_(Eigen::Vector3d::Zero())
    {}

    // 새로운 힘 샘플이 들어올 때마다 호출 (실시간 업데이트)
    // F: 현재 힘 벡터
    // 반환: 최근 1초 동안의 적분 결과 (운동량)
    Eigen::Vector3d update(const Eigen::Vector3d& F) {
        // 누적 적분 업데이트 (Euler 방식)
        integral_ += F * dt_;

        // 1초 전(버퍼에 저장된 값)을 가져옴
        Eigen::Vector3d old_integral = buffer_[current_index_];

        // 원형 버퍼에 현재 누적값을 저장하여, 앞으로 1초 후에 이 값을 빼게 됨
        buffer_[current_index_] = integral_;

        // 버퍼 인덱스 업데이트
        current_index_ = (current_index_ + 1) % window_samples_;

        // 1초 동안의 적분 결과: 현재 누적값과 1초 전 누적값의 차이
        return integral_ - old_integral;
    }

private:
    double dt_;                              // 고정 시간 간격
    size_t window_samples_;                  // 1초 동안의 샘플 개수
    std::vector<Eigen::Vector3d> buffer_;      // 원형 버퍼 (과거 1초의 누적 적분값 저장)
    size_t current_index_;                   // 원형 버퍼 인덱스
    Eigen::Vector3d integral_;               // 현재까지의 누적 적분값
};

#endif //REALTIMEINTEGRATOR_HPP
