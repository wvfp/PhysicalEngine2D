#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>

#ifdef DEBUG
#define DEBUG(x) std::cout << x << std::endl
#else
#define DEBUG(x)
#endif // DEBUG

struct {
	GLuint WINDWOW_WIDTH;
	GLuint WINDOW_HEIGHT;
}window_size = { 1600,1200 };

//Global variables
GLFWwindow* window = nullptr;
ImFont* font = nullptr;

// 窗口大小回调函数
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
// 键盘输入回调函数
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
// 鼠标输入回调函数
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
// 鼠标移动回调函数
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
// 鼠标滚轮回调函数
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
// 窗口关闭回调函数
void window_close_callback(GLFWwindow* window);

// 主函数
int main()
{
	// 初始化 GLFW
	if (!glfwInit())
		return -1;

	// 设置 OpenGL 版本为 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// 创建窗口
	window = glfwCreateWindow(1600, 1200, "Examples For Physical Engine 2D (PE2D)", NULL, NULL);
	if (window == NULL)
	{
		printf("Failed to create GLFW window\n");
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetWindowCloseCallback(window, window_close_callback);

	// 初始化 ImGui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	ImGui::StyleColorsClassic();
	// 初始化 ImGui GLFW 和 OpenGL3 后端
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330 core");
	
	// 设置字体
	font = io.Fonts->AddFontFromFileTTF("res/fonts/Kingnammm-Maiyuan-II-Regular-2.ttf", 16.0f, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());

	// 主循环
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();

		// 开始新的 ImGui 帧
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		ImGui::PushFont(font);
		ImGui::Begin("Hello, world!");
		ImGui::Text("This is some useful text.");
		ImGui::Text("Window size: %d x %d", window_size.WINDWOW_WIDTH, window_size.WINDOW_HEIGHT);
		ImGui::End();
		ImGui::PopFont();
		// 渲染 ImGui
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		// 交换缓冲区
		glfwSwapBuffers(window);
	}

	// 清理 ImGui 和 GLFW
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// 当窗口大小改变时，调整视口大小
	window_size.WINDWOW_WIDTH = width;
	window_size.WINDOW_HEIGHT = height;
	glViewport(0, 0, width, height);
	DEBUG("Window size changed: " << width << ", " << height);
}
// 键盘输入回调函数
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
}
// 鼠标输入回调函数
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		DEBUG("Mouse clicked at: " << xpos << ", " << ypos);
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		DEBUG("Right mouse button clicked");
	}
	else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
		DEBUG("Middle mouse button clicked");
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		DEBUG("Left mouse button released");
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
		DEBUG("Right mouse button released");
	}
	else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE) {
		DEBUG("Middle mouse button released");
	}
}
// 鼠标移动回调函数
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
	// 获取鼠标位置
	double x, y;
	glfwGetCursorPos(window, &x, &y);
	DEBUG("Mouse moved to: " << x << ", " << y);
}
// 鼠标滚轮回调函数
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	// 获取滚轮滚动的距离
	DEBUG("Mouse scrolled: " << xoffset << ", " << yoffset);
	// 这里可以根据滚动的距离来进行缩放等操作
	// 例如，缩放视口大小
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	int new_width = width + static_cast<int>(xoffset * 10);
	int new_height = height + static_cast<int>(yoffset * 10);
	glfwSetWindowSize(window, new_width, new_height);
	DEBUG("Window size changed to: " << new_width << ", " << new_height);
}
// 窗口关闭回调函数
void window_close_callback(GLFWwindow* window) {
	// 这里可以进行一些清理操作
	DEBUG("Window closed");
	glfwSetWindowShouldClose(window, GLFW_TRUE);
}