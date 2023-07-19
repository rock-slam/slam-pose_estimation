#ifndef _POSE_ESTIMATION_STREAM_ALIGNMENT_VERIFIER_HPP
#define _POSE_ESTIMATION_STREAM_ALIGNMENT_VERIFIER_HPP

#include <map>
#include <string>
#include <iostream>

namespace pose_estimation
{

  /**
   * Evaluates if there are streams with alignment failures present.
   */
  class StreamAlignmentVerifier
  {
  public:
    StreamAlignmentVerifier() : aligner_last_verified_usec(0), verification_interval(2.0), drop_rate_warning(0.5),
                                drop_rate_critical(1.0), min_new_samples(5) {}

    virtual ~StreamAlignmentVerifier() {}

    template <class StreamAlignerStatus>
    void verifyStreamAlignerStatus(const StreamAlignerStatus &status, unsigned &streams_with_alignment_failures,
                                   unsigned &streams_with_critical_alignment_failures)
    {
      if (static_cast<double>(status.time.microseconds - aligner_last_verified_usec) / 1000000.0 > verification_interval)
      {
        streams_with_alignment_failures = 0;
        streams_with_critical_alignment_failures = 0;
        for (const auto &stream_status : status.streams)
        {
          // no samples received
          if (aligner_samples_received[stream_status.name] == 0)
          {
            aligner_samples_received[stream_status.name] = stream_status.samples_received;
            continue;
          }

          size_t new_samples_received = stream_status.samples_received - aligner_samples_received[stream_status.name];
          size_t samples_dropped = stream_status.samples_dropped_buffer_full + stream_status.samples_dropped_late_arriving + stream_status.samples_backward_in_time;
          size_t new_samples_dropped = samples_dropped - aligner_samples_dropped[stream_status.name];

          if (new_samples_received > min_new_samples)
          {
            // check if more than 50% of samples are dropped
            double drop_rate = (double)new_samples_dropped / (double)new_samples_received;
            if (drop_rate >= drop_rate_critical)
            {
              streams_with_critical_alignment_failures++;
              std::cerr << "Critical transformation alignment failure in stream " << stream_status.name << ". " << drop_rate * 100.0 << "% of all samples were dropped in the last " << verification_interval << " seconds." << std::endl;
            }
            else if (drop_rate > drop_rate_warning)
            {
              streams_with_alignment_failures++;
              std::cerr << "Transformation alignment failure in stream " << stream_status.name << ". " << drop_rate * 100.0 << "% of all samples were dropped in the last " << verification_interval << " seconds." << std::endl;
            }
          }
          else
          {
            std::cout << "To few samples received to validate the drop rate in stream " << stream_status.name << std::endl;
          }

          aligner_samples_received[stream_status.name] = stream_status.samples_received;
          aligner_samples_dropped[stream_status.name] = samples_dropped;
        }
        aligner_last_verified_usec = status.time.microseconds;
      }
    }

    template <class StreamAlignerStatus>
    void verifyStreamAlignerStatus(const StreamAlignerStatus &status, unsigned &streams_with_alignment_failures)
    {
      unsigned aux;
      verifyStreamAlignerStatus(status, streams_with_alignment_failures, aux);
    }

    /**
     * Sets verification interval in seconds
     */
    void setVerificationInterval(double verification_interval) { this->verification_interval = verification_interval; }
    double getVerificationInterval() { return verification_interval; }
    /**
     * Drop rate in % in [0,1] to count as warning.
     */
    void setDropRateWarningThreshold(double drop_rate_warning) { this->drop_rate_warning = drop_rate_warning; }
    double getDropRateWarningThreshold() { return drop_rate_warning; }
    /**
     * Drop rate in % in [0,1] to count as critical.
     */
    void setDropRateCriticalThreshold(double drop_rate_critical) { this->drop_rate_critical = drop_rate_critical; }
    double getDropRateCriticalThreshold() { return drop_rate_critical; }

  protected:
    std::map<std::string, size_t> aligner_samples_received;
    std::map<std::string, size_t> aligner_samples_dropped;
    int64_t aligner_last_verified_usec;
    double verification_interval;
    double drop_rate_warning;
    double drop_rate_critical;
    unsigned min_new_samples;
  };

}

#endif