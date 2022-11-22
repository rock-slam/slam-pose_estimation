// #include "StreamAlignmentVerifier.hpp"

// using namespace pose_estimation;

// StreamAlignmentVerifier::StreamAlignmentVerifier()
// {
//     verification_interval = 2.0;
//     drop_rate_warning = 0.5;
//     drop_rate_critical = 1.0;
//     aligner_last_verified.microseconds = 0;
//     min_new_samples = 5;
// }

// void StreamAlignmentVerifier::verifyStreamAlignerStatus(const aggregator::StreamAlignerStatus& status,
//                                                         unsigned int& streams_with_alignment_failures,
//                                                         unsigned int &streams_with_critical_alignment_failures)
// {
//     if((status.time - aligner_last_verified).toSeconds() > verification_interval)
//     {
//         streams_with_alignment_failures = 0;
//         streams_with_critical_alignment_failures = 0;
//         for(std::vector<aggregator::StreamStatus>::const_iterator it = status.streams.begin();
//             it != status.streams.end(); it++)
//         {
//             // no samples received
//             if(aligner_samples_received[it->name] == 0)
//             {
//                 aligner_samples_received[it->name] = it->samples_received;
//                 continue;
//             }

//             size_t new_samples_received = it->samples_received - aligner_samples_received[it->name];
//             size_t samples_dropped = it->samples_dropped_buffer_full + it->samples_dropped_late_arriving + it->samples_backward_in_time;
//             size_t new_samples_dropped = samples_dropped - aligner_samples_dropped[it->name];

//             if(new_samples_received > min_new_samples)
//             {
//                 // check if more than 50% of samples are dropped
//                 double drop_rate = (double)new_samples_dropped / (double)new_samples_received;
//                 if(drop_rate >= drop_rate_critical)
//                 {
//                     streams_with_critical_alignment_failures++;
//                     LOG_ERROR_S << "Critical transformation alignment failure in stream " << it->name <<
//                                             ". " << drop_rate * 100.0 << "% of all samples were dropped in the last " <<
//                                             verification_interval << " seconds.";
//                 }
//                 else if(drop_rate > drop_rate_warning)
//                 {
//                     streams_with_alignment_failures++;
//                     LOG_ERROR_S << "Transformation alignment failure in stream " << it->name <<
//                                             ". " << drop_rate * 100.0 << "% of all samples were dropped in the last " <<
//                                             verification_interval << " seconds.";
//                 }
//             }
//             else
//             {
//                 LOG_INFO_S << "To few samples received to validate the drop rate in stream " << it->name;
//             }

//             aligner_samples_received[it->name] = it->samples_received;
//             aligner_samples_dropped[it->name] = samples_dropped;
//         }
//         aligner_last_verified = status.time;
//     }
// }

// void StreamAlignmentVerifier::verifyStreamAlignerStatus(const aggregator::StreamAlignerStatus& status, unsigned int& streams_with_alignment_failures)
// {
//     unsigned aux;
//     verifyStreamAlignerStatus(status, streams_with_alignment_failures, aux);
// }