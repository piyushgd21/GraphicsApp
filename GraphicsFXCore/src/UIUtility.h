///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	UIUtility.h
//	Description	:
//		Defines simple Vulkan UI drawing for text and popup windows.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "InputUtility.h"
#include "MathUtility.h"
#include "SetupUtility.h"

#include <vulkan/vulkan.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace graphicsFX
{
    //Defines simple Vulkan UI drawing for text and popup windows.
    class UIUtility
    {

        public:

            //Defines a popup button callback.
            using PopupCallback = std::function<void(int)>;

            //Stores one popup slider.
            struct PopupSlider
            {
                std::string m_label; //Stores the slider label.
                float m_minimum{ 0.0f }; //Stores the minimum slider value.
                float m_maximum{ 1.0f }; //Stores the maximum slider value.
                float m_value{ 0.0f }; //Stores the current slider value.
            };

            //Defines a popup slider callback.
            using PopupSliderCallback = std::function<void(int, float)>;

            //Defines a popup button callback with slider values.
            using PopupSliderButtonCallback = std::function<void(int, const std::vector<PopupSlider> &)>;

            //Creates the UI utility.
            UIUtility(void) = default;

            //Destroys the UI utility.
            ~UIUtility(void) = default;

            UIUtility(const UIUtility &) = delete;
            UIUtility &operator=(const UIUtility &) = delete;

            //Initializes UI GPU resources.
            void Init(VkDevice devicePtrGPU, VkPhysicalDevice physicalDevicePtrGPU, VkRenderPass renderPassPtrGPU, VkExtent2D swapChainExtent, std::size_t frameCount, const UISetup &uiSetup);

            //Starts a new UI frame.
            void BeginFrame(VkExtent2D swapChainExtent, std::size_t frameIndex);

            //Prints text to the Visual Studio debug window.
            static void Print(std::string data);

            //Draws UI text.
            void DrawText(float x, float y, const std::string &text, const Float4 &color, float size);

            //Draws UI text projected from a 3D world position.
            bool DrawText3D(const std::string &text, const Float3 &worldPosition, const Float4x4 &view, const Float4x4 &projection, const Float2 &screenOffset, const Float4 &color, float size);

            //Draws a UI line.
            void DrawLine(float startX, float startY, float endX, float endY, const Float4 &color, float thickness);

            //Draws a glass UI panel.
            void DrawGlassPanel(float x, float y, float width, float height);

            //Shows a stored popup window.
            void ShowPopup(const std::string &title, const std::string &message, const std::vector<std::string> &buttons, PopupCallback callback);

            //Shows a stored popup window with sliders.
            void ShowPopup(
                const std::string &title,
                const std::string &message,
                const std::vector<std::string> &buttons,
                const std::vector<PopupSlider> &sliders,
                PopupSliderButtonCallback buttonCallback,
                PopupSliderCallback sliderCallback = nullptr
            );

            //Updates the stored popup title and body text.
            void UpdatePopupText(const std::string &title, const std::string &message);

            //Hides the stored popup window.
            void HidePopup(void);

            //Updates UI input state.
            void UpdateInput(const InputUtility &inputUtility);

            //Draws stored UI windows.
            void DrawWindows(void);

            //Returns whether a stored popup is visible.
            bool PopupVisible(void) const;

            //Draws UI geometry to a command buffer.
            void Show(VkCommandBuffer commandBufferPtrGPU);

            //Releases only the graphics pipeline.
            void CleanPipeline(void);

            //Recreates the pipeline after swapchain changes.
            void RefreshPipeline(VkRenderPass renderPassPtrGPU, VkExtent2D swapChainExtent);

            //Releases all UI GPU resources.
            void Clean(void);

        private:

            //Stores one UI vertex.
            struct UIVertex
            {
                Float2 m_position; //Stores screen position.
                Float4 m_color; //Stores vertex color.

                //Returns the Vulkan vertex binding layout.
                static VkVertexInputBindingDescription BindingDescription(void);

                //Returns the Vulkan vertex attribute layout.
                static std::array<VkVertexInputAttributeDescription, 2> AttributeDescriptions(void);
            };

            //Stores UI shader constants.
            struct UIShaderConstants
            {
                Float2 m_screenSize; //Stores the current screen size.
            };

            //Stores one cached font glyph.
            struct FontGlyph
            {
                char m_character{ ' ' }; //Stores the glyph character.
                int m_pixelHeight{ 0 }; //Stores the requested font pixel height.
                int m_width{ 0 }; //Stores glyph bitmap width.
                int m_height{ 0 }; //Stores glyph bitmap height.
                int m_advance{ 0 }; //Stores cursor advance after drawing.
                std::vector<uint8_t> m_alpha; //Stores glyph alpha pixels.
            };

            //Creates the UI graphics pipeline.
            void CreatePipeline(void);

            //Creates the UI vertex and index buffers.
            void CreateBuffers(void);

            //Creates a Vulkan buffer and memory.
            void CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &bufferPtrGPU, VkDeviceMemory &bufferMemoryPtrGPU) const;

            //Creates a Vulkan shader module.
            VkShaderModule CreateShaderModule(const std::vector<char> &code) const;

            //Finds a matching memory type.
            uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;

            //Loads a compiled shader file.
            std::vector<char> ReadShaderFile(const std::string &relativePath) const;

            //Draws a popup window and returns clicked button index.
            int DrawPopup(
                float &x,
                float &y,
                float width,
                float height,
                const std::string &title,
                const std::string &message,
                const std::vector<std::string> &buttons,
                std::vector<PopupSlider> &sliders,
                const InputUtility &inputUtility
            );

            //Draws popup sliders.
            void DrawPopupSliders(float x, float y, float width, std::vector<PopupSlider> &sliders, const InputUtility &inputUtility);

            //Returns a formatted slider value.
            std::string FormatSliderValue(float value) const;

            //Finds or creates a cached font glyph.
            const FontGlyph &FindGlyph(char character, float size);

            //Builds one font glyph using Windows GDI.
            FontGlyph BuildGlyph(char character, int pixelHeight) const;

            //Draws one cached font glyph.
            void DrawGlyph(float x, float y, const FontGlyph &glyph, const Float4 &color);

            //Returns the rendered text width.
            float TextWidth(const std::string &text, float size);

            //Returns the rendered line height.
            float LineHeight(float size);

            //Returns the wrapped text height.
            float WrappedTextHeight(const std::string &text, float width, float size);

            //Returns the popup height needed for the current content.
            float PopupContentHeight(float width);

            //Adds a filled rectangle.
            void AddRect(float x, float y, float width, float height, const Float4 &color);

            //Adds a rectangle border.
            void AddRectBorder(float x, float y, float width, float height, float thickness, const Float4 &color);

            //Draws wrapped UI text.
            void DrawWrappedText(float x, float y, float width, const std::string &text, const Float4 &color, float size);

            //Returns whether the mouse is inside a rectangle.
            bool MouseInRect(const InputUtility &inputUtility, float x, float y, float width, float height) const;

            //Returns an alpha value suitable for smooth glyph drawing.
            static uint8_t QuantizeAlpha(uint8_t alpha);

            static constexpr std::size_t s_defaultMaxVertices = 80000; //Stores default maximum UI vertices.
            static constexpr std::size_t s_defaultMaxIndices = 120000; //Stores default maximum UI indices.
            static constexpr float s_fontPixelScale = 9.0f; //Stores pixels used for each UI text size.
            static constexpr int s_glyphPadding = 2; //Stores transparent padding around each glyph.
            static constexpr uint8_t s_minimumGlyphAlpha = 10U; //Stores minimum visible glyph alpha.

            VkDevice m_devicePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan logical device.
            VkPhysicalDevice m_physicalDevicePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan physical device.
            VkRenderPass m_renderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the render pass used by the UI pipeline.
            VkExtent2D m_swapChainExtent{}; //Stores the active swapchain size.
            std::size_t m_frameCount{ 0 }; //Stores the number of frames in flight.
            std::size_t m_currentFrame{ 0 }; //Stores the current frame index.

            VkPipelineLayout m_pipelineLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores the UI pipeline layout.
            VkPipeline m_pipelinePtrGPU{ VK_NULL_HANDLE }; //Stores the UI graphics pipeline.
            std::vector<VkBuffer> m_vertexBuffersPtrGPU; //Stores the UI vertex buffers.
            std::vector<VkDeviceMemory> m_vertexBufferMemoriesPtrGPU; //Stores UI vertex buffer memory.
            std::vector<VkBuffer> m_indexBuffersPtrGPU; //Stores the UI index buffers.
            std::vector<VkDeviceMemory> m_indexBufferMemoriesPtrGPU; //Stores UI index buffer memory.
            std::vector<void *> m_vertexMappedPtr; //Stores mapped UI vertex memory.
            std::vector<void *> m_indexMappedPtr; //Stores mapped UI index memory.

            std::vector<UIVertex> m_vertices; //Stores UI vertices for this frame.
            std::vector<uint32_t> m_indices; //Stores UI indices for this frame.
            std::vector<FontGlyph> m_fontGlyphs; //Stores cached Windows font glyphs.
            UISetup m_uiSetup; //Stores loaded UI setup values.
            std::size_t m_maxVertices{ s_defaultMaxVertices }; //Stores maximum UI vertices per frame.
            std::size_t m_maxIndices{ s_defaultMaxIndices }; //Stores maximum UI indices per frame.
            const InputUtility *m_inputUtilityPtr{ nullptr }; //Stores the current frame input utility.
            bool m_popupVisible{ false }; //Stores whether the popup is visible.
            bool m_popupPositionReady{ false }; //Stores whether popup position has been initialized.
            bool m_popupDragging{ false }; //Stores whether the popup title is being dragged.
            float m_popupWidth{ 560.0f }; //Stores popup width.
            float m_popupHeight{ 240.0f }; //Stores popup height.
            std::string m_popupTitle; //Stores popup title text.
            std::string m_popupMessage; //Stores popup body text.
            std::vector<std::string> m_popupButtons; //Stores popup button text.
            std::vector<PopupSlider> m_popupSliders; //Stores popup slider data.
            PopupCallback m_popupCallback; //Stores popup button callback.
            PopupSliderButtonCallback m_popupSliderButtonCallback; //Stores popup button callback with sliders.
            PopupSliderCallback m_popupSliderCallback; //Stores popup slider callback.
            Float2 m_popupPosition{}; //Stores popup screen position.
            Float2 m_popupDragOffset{}; //Stores mouse offset from popup origin during drag.
            int m_popupActiveSliderIndex{ -1 }; //Stores the active popup slider index.
    };
}

