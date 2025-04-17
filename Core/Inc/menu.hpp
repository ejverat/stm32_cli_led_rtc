#pragma once

#include <string>
#include <string_view>

template <typename T> class OutBuffer {
public:
  void write(const std::string &text) {
    static_cast<T *>(this)->do_write(text);
  }
};

template <typename OutBuffer> class Menu {
public:
  Menu() = default;
  Menu(OutBuffer bufferOut) : bufferOut(bufferOut) {}
  void show(void) { bufferOut.write("Text example of menu\r\n"); }

private:
  OutBuffer bufferOut;
  const std::string text = "Text example of menu\r\n";
};
