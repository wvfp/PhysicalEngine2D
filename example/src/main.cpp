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

// ���ڴ�С�ص�����
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
// ��������ص�����
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
// �������ص�����
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
// ����ƶ��ص�����
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
// �����ֻص�����
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
// ���ڹرջص�����
void window_close_callback(GLFWwindow* window);

// ������
int main()
{
	// ��ʼ�� GLFW
	if (!glfwInit())
		return -1;

	// ���� OpenGL �汾Ϊ 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// ��������
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

	// ��ʼ�� ImGui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	ImGui::StyleColorsClassic();
	// ��ʼ�� ImGui GLFW �� OpenGL3 ���
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330 core");
	
	// ��������
	font = io.Fonts->AddFontFromFileTTF("res/fonts/Kingnammm-Maiyuan-II-Regular-2.ttf", 16.0f, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());

	// ��ѭ��
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();

		// ��ʼ�µ� ImGui ֡
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		ImGui::PushFont(font);
		ImGui::Begin("Hello, world!");
		ImGui::Text("This is some useful text.");
		ImGui::Text("Window size: %d x %d", window_size.WINDWOW_WIDTH, window_size.WINDOW_HEIGHT);
		ImGui::End();
		ImGui::PopFont();
		// ��Ⱦ ImGui
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		// ����������
		glfwSwapBuffers(window);
	}

	// ���� ImGui �� GLFW
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// �����ڴ�С�ı�ʱ�������ӿڴ�С
	window_size.WINDWOW_WIDTH = width;
	window_size.WINDOW_HEIGHT = height;
	glViewport(0, 0, width, height);
	DEBUG("Window size changed: " << width << ", " << height);
}
// ��������ص�����
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
}
// �������ص�����
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
// ����ƶ��ص�����
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
	// ��ȡ���λ��
	double x, y;
	glfwGetCursorPos(window, &x, &y);
	DEBUG("Mouse moved to: " << x << ", " << y);
}
// �����ֻص�����
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	// ��ȡ���ֹ����ľ���
	DEBUG("Mouse scrolled: " << xoffset << ", " << yoffset);
	// ������Ը��ݹ����ľ������������ŵȲ���
	// ���磬�����ӿڴ�С
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	int new_width = width + static_cast<int>(xoffset * 10);
	int new_height = height + static_cast<int>(yoffset * 10);
	glfwSetWindowSize(window, new_width, new_height);
	DEBUG("Window size changed to: " << new_width << ", " << new_height);
}
// ���ڹرջص�����
void window_close_callback(GLFWwindow* window) {
	// ������Խ���һЩ�������
	DEBUG("Window closed");
	glfwSetWindowShouldClose(window, GLFW_TRUE);
}