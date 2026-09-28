
class Application
{
    public: void run();

    private: void init();
        void mainLoop();
        void shutdown();

    private: std::unique_ptr<Window> window;
        std::unique_ptr<InputSystem> inputSystem;
        std::unique_ptr<Scene> scene;
        std::unique_ptr<Renderer> renderer;
        std::unique_ptr<XrSystem> xrSystem;
        std::unique_ptr<LookingGlassSystem> lookingGlassSystem;
};

class Window
{
    public: Window(int width, int height, const std::string& title);
        ~Window();

        bool shouldClose() const;
        void pollEvents();

        bool isMinimized() const;
        bool wasResized() const;
        void resetResizeFlag();

        int width() const;
        int height() const;

        void* nativeHandle() const;

    private: int m_width;
        int m_height;
        std::string m_title;
        bool m_shouldClose = false;
        bool m_minimized = false;
        bool m_resized = false;
};

struct Vertex
{
    float3 position;
    float3 normal;
    float2 uv;
    float4 tangent;
};

class MeshAsset
{
    public: std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;

        void loadFromFile(const std::string& path);
};

class MeshGpuResources
{
    public: BufferHandle vertexBuffer;
        BufferHandle indexBuffer;
        BufferHandle normalBuffer;
        BufferHandle tangentBuffer;

        void upload(IRenderBackend& backend, const MeshAsset& asset);
};

class RenderObject
{
    public: std::shared_ptr<MeshAsset> mesh;
        MeshGpuResources gpu;
        Transform transform;

        void update(float dt);
};

class Scene
{
    public: void init();
        void update(float dt);
        void shutdown();

        std::vector<std::shared_ptr<RenderObject>>& objects();
        Camera& activeCamera();

    private: std::vector<std::shared_ptr<RenderObject>> m_objects;
        Camera m_camera;
};

class IRenderBackend
{
    public: virtual ~IRenderBackend() = default;

        virtual void initialize(const Window& window) = 0;
        virtual void shutdown() = 0;

        virtual void beginFrame() = 0;
        virtual void endFrame() = 0;
        virtual void resize(int width, int height) = 0;

        virtual std::unique_ptr<ICommandBuffer> createCommandBuffer() = 0;
        virtual std::unique_ptr<IBuffer> createBuffer(const BufferDesc& desc) = 0;
        virtual std::unique_ptr<ITexture> createTexture(const TextureDesc& desc) = 0;
        virtual std::unique_ptr<IPipeline> createPipeline(const PipelineDesc& desc) = 0;
        virtual std::unique_ptr<ISwapchain> createSwapchain(const SwapchainDesc& desc) = 0;
};

class VulkanBackend : public IRenderBackend
{
    public: void initialize(const Window& window) override;
        void shutdown() override;
        void beginFrame() override;
        void endFrame() override;
        void resize(int width, int height) override;

        std::unique_ptr<ICommandBuffer> createCommandBuffer() override;
        std::unique_ptr<IBuffer> createBuffer(const BufferDesc& desc) override;
        std::unique_ptr<ITexture> createTexture(const TextureDesc& desc) override;
        std::unique_ptr<IPipeline> createPipeline(const PipelineDesc& desc) override;
        std::unique_ptr<ISwapchain> createSwapchain(const SwapchainDesc& desc) override;

    private: VkInstance instance = VK_NULL_HANDLE;
        VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
        VkDevice device = VK_NULL_HANDLE;
        VkQueue graphicsQueue = VK_NULL_HANDLE;
        VkQueue presentQueue = VK_NULL_HANDLE;
};

class PresentationManager
{
    public: void initialize(IRenderBackend& backend, Window& window);
        void shutdown();

        void presentDesktop();
        void presentXr(XrSystem& xrSystem);
        void presentLookingGlass(LookingGlassSystem& lgSystem);

        void resize(int width, int height);

    private: std::unique_ptr<ISwapchain> desktopSwapchain;
};

class Renderer
{
    public: void init(Window& window, RenderBackendType backendType);
        void drawFrame(Scene& scene);
        void resize(int width, int height);
        void shutdown();

        void setXrSystem(XrSystem* xrSystem);
        void setLookingGlassSystem(LookingGlassSystem* lgSystem);

    private: std::unique_ptr<IRenderBackend> backend;
        std::unique_ptr<PresentationManager> presentationManager;
        std::unique_ptr<RenderGraph> renderGraph;
        FrameResources frameResources[MAX_FRAMES_IN_FLIGHT];
};

