#ifndef _POSE_ESTIMATION_STREAM_ALIGNMENT_VERIFIER_HPP
#define _POSE_ESTIMATION_STREAM_ALIGNMENT_VERIFIER_HPP

#include <map>
#include <base/Time.hpp>
#include <string>
#include <aggregator/StreamAlignerStatus.hpp>

namespace pose_estimation
{

class StreamAlignmentVerifier
{
public:
    StreamAlignmentVerifier();
    virtual ~StreamAlignmentVerifier() {}
    void verifyStreamAlignerStatus(const aggregator::StreamAlignerStatus &status, unsigned &streams_with_alignment_failures);

    void setVerificationInterval(double verification_interval) {this->verification_interval = verification_interval;}
    double getVerificationInterval() {return verification_interval;}
    void setDropRateThreshold(double drop_rate_threshold) {this->drop_rate_threshold = drop_rate_threshold;}
    double getDropRateThreshold() {return drop_rate_threshold;};

protected:
    std::map<std::string, size_t> aligner_samples_received;
    std::map<std::string, size_t> aligner_samples_dropped;
    base::Time aligner_last_verified;
    double verification_interval;
    double drop_rate_threshold;
};

}

#endif