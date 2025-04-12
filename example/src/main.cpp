#undef glm
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <PhysicalEngine2D.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <shader/shader.hpp>
#include <camera/camera.hpp>
#include <tuple>
#ifdef DEBUG_MODE
#define DEBUG(x) std::cout << x << std::endl
#else
#define DEBUG(x)
#endif // DEBUG

struct {
	GLfloat WINDWOW_WIDTH;
	GLfloat WINDOW_HEIGHT;
}window_size = { 1600,1200 };
//Global variables
GLFWwindow* window = nullptr;
ImFont* font = nullptr;
std::vector<std::tuple<GLuint, GLuint, GLuint>> vaos;
std::vector<std::tuple<GLuint, GLuint, GLuint>> aabb_Vaos;
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
// 渲染AABB
void initRender(const PE2D::AABB* aabb, GLuint id);
void initRender(const PE2D::Shape* shape, GLuint id);
Camera camera(glm::vec3(0.0f, 0.0f, 100.0f));
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

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		glfwTerminate();
		return -1;
	}

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

	PE2D::World world(PE2D::Vector2D(0, -9.8), 0.02);
	unsigned int box = PE2D::RigidBody::makeRigidBody();
	unsigned int ground = PE2D::RigidBody::makeRigidBody();
	world.addObject(box);
	world.addObject(ground);
	auto obj = PE2D::Object::ID_Map[box];
	auto g = PE2D::Object::ID_Map[ground];
	PE2D::Shape* g_shape = new PE2D::Rectangle(PE2D::Vector2D(-100, -20), PE2D::Vector2D(100, -22));
	PE2D::Shape* b_shape = new PE2D::Rectangle(PE2D::Vector2D(0, 4), PE2D::Vector2D(4, 2));
	obj->setShape(b_shape);
	g->setShape(g_shape);
	g->setFixed(true);
	Shader aabb_shader = Shader("res/shaders/aabb.vert", "res/shaders/aabb.frag");
	Shader obj_shader = Shader("res/shaders/obj.vert", "res/shaders/obj.frag");
	initRender(obj->getShape(), box);
	initRender(obj->getAABB(), box);
	initRender(g->getShape(), ground);
	initRender(g->getAABB(), ground);
	//// 主循环
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();

		//obj->applyForce(PE2D::Vector2D(0, 9.7), obj->getCentroid());
		world.update();

		// 开始新的 ImGui 帧
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGui::PushFont(font);
		ImGui::Begin("Hello, world!");
		ImGui::Text("This is some useful text.");
		ImGui::Text("Obj pos: %f x %f", obj->getPosition().x(), obj->getPosition().y());
		ImGui::End();
		ImGui::PopFont();

		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
		glClear(GL_COLOR_BUFFER_BIT);

		aabb_shader.use();
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), window_size.WINDWOW_WIDTH / window_size.WINDOW_HEIGHT, 0.1f, 1000.0f);
		glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 model = glm::mat4(1.0f);
		for (auto i : aabb_Vaos) {
			model = glm::mat4(1.0);
			PE2D::Vector2D pos = PE2D::Object::ID_Map[std::get<2>(i)]->getPosition();
			model = glm::translate(model, glm::vec3(pos.x(), pos.y(), 0.0f));
			aabb_shader.use();
			aabb_shader.setMat4("projection", projection);
			aabb_shader.setMat4("view", view);
			aabb_shader.setMat4("model", model);
			glBindVertexArray(std::get<0>(i));
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glLineWidth(8.0);
			glDrawArrays(GL_LINE_STRIP, 0, std::get<1>(i));
			glBindVertexArray(0);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glLineWidth(1.0);
		}
		for (auto i : vaos) {
			model = glm::mat4(1.0);
			PE2D::Vector2D pos = PE2D::Object::ID_Map[std::get<2>(i)]->getPosition();
			model = glm::translate(model, glm::vec3(pos.x(), pos.y(), 0.0f));
			obj_shader.use();
			obj_shader.setMat4("projection", projection);
			obj_shader.setMat4("view", view);
			obj_shader.setMat4("model", model);
			glBindVertexArray(std::get<0>(i));
			glDrawArrays(GL_TRIANGLE_STRIP, 0, std::get<1>(i));
			glBindVertexArray(0);
		}
		// 渲染 ImGui
		ImGui::Render();
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
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
	}
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
		camera.ProcessKeyboard(CAMERA_UP, 0.03);
	}
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
		camera.ProcessKeyboard(CAMERA_DOWM, 0.03);
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
		camera.ProcessKeyboard(CAMERA_LEFT, 0.03);
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
		camera.ProcessKeyboard(CAMERA_RIGHT, 0.03);
	}
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		camera.ProcessKeyboard(CAMERA_FORWARD, 0.03);
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
		camera.ProcessKeyboard(CAMERA_BACKWARD, 0.03);
	}
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
	camera.ProcessMouseScroll(yoffset);
}// 窗口关闭回调函数
void window_close_callback(GLFWwindow* window) {
	// 这里可以进行一些清理操作
	DEBUG("Window closed");
	glfwSetWindowShouldClose(window, GLFW_TRUE);
}
void initRender(const PE2D::AABB* aabb, GLuint id) {
	std::vector<float> pos;
	auto br = aabb->getBottomRight() - aabb->getCenter();
	auto tl = aabb->getTopLeft() - aabb->getCenter();
	std::vector<float> vertices = {
		br.x(),br.y(),0.0f,
		br.x(),tl.y(),0.0f,
		tl.x(),tl.y(),0.0f,
		tl.x(),br.y(),0.0f
	};
	vertices.push_back(vertices[0]);
	vertices.push_back(vertices[1]);
	vertices.push_back(vertices[2]);
	//vertices = {
	//	-0.5,-0.5,0.0,
	//	0.5,-0.5,0.0,
	//	0.5,0.5,0.0,
	//	-0.5,0.5,0.0
	//};
	GLuint VAO, VBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	aabb_Vaos.push_back(std::make_tuple(VAO, GLuint(vertices.size()), id));
}
// 生成圆形顶点数据（2D）
//std::vector<glm::vec2> generateCircleVertices(glm::vec2 center, float radius, int segments = 36) {
//	std::vector<glm::vec2> vertices;
//	vertices.push_back(center); // 圆心（第一个顶点）
//
//	for (int i = 0; i <= segments; ++i) {
//		float angle = 2.0f * glm::pi<float>() * i / segments;
//		vertices.push_back({
//			center.x + radius * cos(angle), // x
//			center.y + radius * sin(angle)  // y
//			});
//	}
//	return vertices;
//}
//unsigned int VAO, VBO;

//void initCircle(glm::vec2 pos, float r) {
//	// 生成顶点数据
//	auto vertices = generateCircleVertices(pos, r);
//
//	// 创建 VAO 和 VBO
//	glGenVertexArrays(1, &VAO);
//	glGenBuffers(1, &VBO);
//
//	// 绑定 VAO 和 VBO
//	glBindVertexArray(VAO);
//	glBindBuffer(GL_ARRAY_BUFFER, VBO);
//	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_STATIC_DRAW);
//
//	// 设置顶点属性指针（2D 坐标）
//	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
//	glEnableVertexAttribArray(0);
//
//	// 解绑
//	glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glBindVertexArray(0);
//}
//void renderCircle() {
//	glBindVertexArray(VAO);
//	glDrawArrays(GL_TRIANGLE_FAN, 0, 36 + 2); // +2（圆心 + 闭合点）
//	glBindVertexArray(0);
//}
void initRender(const PE2D::Shape* shape, GLuint id) {
	std::vector<float> vertices;
	//if (!shape->isUnknown() || !shape->isCircle() || !shape->isCapsule()) {
	for (auto i : shape->getVertices()) {
		PE2D::Vector2D p = i;
		p -= shape->getCentroid();
		vertices.push_back(p.x());
		vertices.push_back(p.y());
		vertices.push_back(0.0f);
	}
	vertices.push_back(vertices[0]);
	vertices.push_back(vertices[1]);
	vertices.push_back(vertices[2]);
	//}
	//std::vector<GLuint> indices;
	//根据virtices中的顶点数，添加EBO索引
	//indices.push_back(0);
	//for (GLuint i = 1; i < vertices.size() - 1; i++) {
		//indices.push_back(i);
		//indices.push_back(i + 1);
	//}
	//indices.push_back(vertices.size()-1);
	//indices.push_back(1);
	//s_EBO_Size = indices.size();
	GLuint VAO, VBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,s_EBO);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	vaos.push_back(std::make_tuple(VAO, GLuint(vertices.size()), id));
}