#undef glm
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <shader/shader.hpp>
#include <camera/camera.hpp>
#include <PhysicalEngine2D.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// 顶点数据
float vertices[] = {
	-0.5f, -0.5f, 0.0f,
	 0.5f, -0.5f, 0.0f,
	 0.0f,  0.5f, 0.0f
};
Camera camera(glm::vec3(0.0f, 0.0f, 15.0f));
GLuint aabbVAO, aabbVBO;
void initVertices(const PE2D::AABB* aabb) {
	std::vector<float> pos;
	auto br = aabb->getBottomRight();
	auto tl = aabb->getTopLeft();
	std::vector<float> vertices = {
		br.x(),br.y(),
		br.x(),tl.y(),
		tl.x(),tl.y(),
		tl.x(),br.y(),
	};
	vertices = {
		0.5,0.5,
		1.0,0.5,
		0.5,1.0,
		1.0,1.0
	};
	glGenVertexArrays(1, &aabbVAO);
	glGenBuffers(1, &aabbVBO);

	glBindVertexArray(aabbVAO);

	glBindBuffer(GL_ARRAY_BUFFER, aabbVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}
int main() {
	// 初始化 GLFW
	if (!glfwInit()) {
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return -1;
	}

	// 设置 GLFW 窗口属性
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// 创建窗口
	GLFWwindow* window = glfwCreateWindow(800 * 2, 600 * 2, "IMGUI with OpenGL", nullptr, nullptr);
	if (!window) {
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	// 使窗口的上下文成为当前线程的主上下文
	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		glfwTerminate();
		return -1;
	}
	// 初始化 ImGui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330 core");

	// 设置视口
	glViewport(0, 0, 800 * 2, 600 * 2);

	PE2D::World world(PE2D::Vector2D(0, -9.8), 0.03);
	unsigned int box = PE2D::RigidBody::makeRigidBody();
	unsigned int ground = PE2D::RigidBody::makeRigidBody();
	world.addObject(box);
	world.addObject(ground);
	auto obj = PE2D::Object::ID_Map[box];
	auto g = PE2D::Object::ID_Map[ground];
	PE2D::Shape* g_shape = new PE2D::Rectangle(PE2D::Vector2D(-100, 4), PE2D::Vector2D(100, -4));
	PE2D::Shape* b_shape = new PE2D::Rectangle(PE2D::Vector2D(0, 4), PE2D::Vector2D(3, 3));
	obj->setShape(b_shape);
	g->setShape(g_shape);

	// 生成 VAO 和 VBO
	unsigned int VAO, VBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	initVertices(g->getAABB());
	Shader obj_shader = Shader("res/shaders/obj.vert", "res/shaders/obj.frag");
	Shader aabb_shader = Shader("res/shaders/aabb.vert", "res/shaders/aabb.frag");
	// 主循环
	while (!glfwWindowShouldClose(window)) {
		obj->applyForce(PE2D::Vector2D(0, 9.79), obj->getCentroid());
		world.update();
		// 处理输入
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
			glfwSetWindowShouldClose(window, true);
		}
		if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
			camera.ProcessKeyboard(CAMERA_UP, 0.03);
		}
		// 开始 ImGui 新帧
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// ImGui 窗口示例
		ImGui::Begin("My ImGui Window");
		ImGui::Text("Obj pos: %f - %f", obj->getPosition().x(), obj->getPosition().y());
		ImGui::End();

		// 清除颜色缓冲区
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		// 渲染 OpenGL 物体（三角形）
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), 800.0f / 600.0f, 0.1f, 1000.0f);
		glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 model = glm::translate(glm::mat4(1.0), glm::vec3(obj->getPosition().x(), obj->getPosition().y(), 0));
		model = glm::scale(model, glm::vec3(2, 2, 0));
		obj_shader.use();
		obj_shader.setMat4("projection", projection);
		obj_shader.setMat4("view", view);
		obj_shader.setMat4("model", model);
		glBindVertexArray(VAO);
		glDrawArrays(GL_TRIANGLES, 0, 3);
		glBindVertexArray(0);
		aabb_shader.use();
		aabb_shader.setMat4("projection", projection);
		aabb_shader.setMat4("view", view);
		aabb_shader.setMat4("model", model);
		glBindVertexArray(aabbVAO);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glBindVertexArray(0);
		// 渲染 ImGui 并交换缓冲区
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// 清理资源
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glDeleteBuffers(1, &VBO);
	glDeleteVertexArrays(1, &VAO);

	glfwTerminate();

	return 0;
}