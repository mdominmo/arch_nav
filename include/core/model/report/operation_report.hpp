#ifndef ARCH_NAV__CORE__MODEL__REPORT__OPERATION_REPORT_HPP_
#define ARCH_NAV__CORE__MODEL__REPORT__OPERATION_REPORT_HPP_

namespace arch_nav::report {

enum class ReportStatus { IN_PROGRESS, COMPLETED, ABORTED, FAILED };

class OperationReport {
 public:
  ReportStatus status() const { return status_; }
  void complete() { status_ = ReportStatus::COMPLETED; }
  void abort()    { status_ = ReportStatus::ABORTED; }
  void fail()     { status_ = ReportStatus::FAILED; }

  OperationReport() = default;
  virtual ~OperationReport() = default;

 private:
  ReportStatus status_{ReportStatus::IN_PROGRESS};
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV__CORE__MODEL__REPORT__OPERATION_REPORT_HPP_
