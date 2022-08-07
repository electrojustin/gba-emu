#include <functional>
#include <mutex>
#include <queue>

namespace Core {

public class Future {
private:
  std::mutex lock;   
  std::queue<std::function<void()>> listeners;
  bool is_available;

  static std::shared_ptr<Future> immediate = nullptr;

public:
  Future();
  Future(Future& future) = delete;
  Future(Future&& future);

  void make_available();

  void register_listener(std::function<void()> listener);

  static std::shared_ptr<Future> immediate_future();
}

} // namespace Core
