///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	UIUtility.cpp
//	Description	:
//		Implements simple Vulkan UI drawing for text and popup windows.
//
//
///////////////////////////////////////////////////////////////////////

#include "UIUtility.h"

#include "PathUtility.h"

#include <Windows.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace graphicsFX
{
    //Returns the Vulkan vertex binding layout.
    VkVertexInputBindingDescription UIUtility::UIVertex::BindingDescription(void)
    {
        VkVertexInputBindingDescription binding{};
        binding.binding = 0;
        binding.stride = static_cast<uint32_t>(sizeof(UIVertex));
        binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        return binding;
    }

    //Returns the Vulkan vertex attribute layout.
    std::array<VkVertexInputAttributeDescription, 2> UIUtility::UIVertex::AttributeDescriptions(void)
    {
        std::array<VkVertexInputAttributeDescription, 2> attributes{};

        attributes[0].binding = 0;
        attributes[0].location = 0;
        attributes[0].format = VK_FORMAT_R32G32_SFLOAT;
        attributes[0].offset = static_cast<uint32_t>(offsetof(UIVertex, m_position));

        attributes[1].binding = 0;
        attributes[1].location = 1;
        attributes[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
        attributes[1].offset = static_cast<uint32_t>(offsetof(UIVertex, m_color));

        return attributes;
    }

    //Initializes UI GPU resources.
    void UIUtility::Init(VkDevice devicePtrGPU, VkPhysicalDevice physicalDevicePtrGPU, VkRenderPass renderPassPtrGPU, VkExtent2D swapChainExtent, std::size_t frameCount, const UISetup &uiSetup)
    {
        if (frameCount == 0U)
        {
            throw std::runtime_error("UI frame count must be greater than zero.");
        }

        m_devicePtrGPU = devicePtrGPU;
        m_physicalDevicePtrGPU = physicalDevicePtrGPU;
        m_renderPassPtrGPU = renderPassPtrGPU;
        m_swapChainExtent = swapChainExtent;
        m_frameCount = frameCount;
        m_uiSetup = uiSetup;
        m_maxVertices = static_cast<std::size_t>(std::max(4, uiSetup.m_maxVertices));
        m_maxIndices = static_cast<std::size_t>(std::max(6, uiSetup.m_maxIndices));
        m_vertices.reserve(m_maxVertices);
        m_indices.reserve(m_maxIndices);

        CreateBuffers();
        CreatePipeline();
    }

    //Starts a new UI frame.
    void UIUtility::BeginFrame(VkExtent2D swapChainExtent, std::size_t frameIndex)
    {
        m_swapChainExtent = swapChainExtent;
        m_currentFrame = frameIndex;
        m_inputUtilityPtr = nullptr;
        m_vertices.clear();
        m_indices.clear();
    }

    //Prints text to the Visual Studio debug window.
    void UIUtility::Print(std::string data)
    {
        OutputDebugStringA(data.c_str());
    }

    //Draws UI text.
    void UIUtility::DrawText(float x, float y, const std::string &text, const Float4 &color, float size)
    {
        float cursorX = x;
        float cursorY = y;
        const float lineHeight = LineHeight(size);

        for (char character : text)
        {
            if (character == '\r')
            {
                continue;
            }

            if (character == '\n')
            {
                cursorX = x;
                cursorY += lineHeight;
                continue;
            }

            if (character == '\t')
            {
                cursorX += TextWidth(" ", size) * 4.0f;
                continue;
            }

            const FontGlyph &glyph = FindGlyph(character, size);
            DrawGlyph(cursorX, cursorY, glyph, color);
            cursorX += static_cast<float>(glyph.m_advance);
        }
    }

    //Draws UI text projected from a 3D world position.
    bool UIUtility::DrawText3D(const std::string &text, const Float3 &worldPosition, const Float4x4 &view, const Float4x4 &projection, const Float2 &screenOffset, const Float4 &color, float size)
    {
        if (text.empty() || m_swapChainExtent.width == 0U || m_swapChainExtent.height == 0U)
        {
            return false;
        }

        const Float4 worldPoint{ worldPosition.x, worldPosition.y, worldPosition.z, 1.0f };
        const Float4 viewPoint = transform(view, worldPoint);
        const Float4 clipPoint = transform(projection, viewPoint);
        if (!std::isfinite(clipPoint.w) || clipPoint.w <= 0.0001f)
        {
            return false;
        }

        const float ndcX = clipPoint.x / clipPoint.w;
        const float ndcY = clipPoint.y / clipPoint.w;
        const float ndcZ = clipPoint.z / clipPoint.w;
        if (!std::isfinite(ndcX) || !std::isfinite(ndcY) || !std::isfinite(ndcZ))
        {
            return false;
        }

        if (ndcX < -1.0f || ndcX > 1.0f || ndcY < -1.0f || ndcY > 1.0f || ndcZ < 0.0f || ndcZ > 1.0f)
        {
            return false;
        }

        const float screenWidth = static_cast<float>(m_swapChainExtent.width);
        const float screenHeight = static_cast<float>(m_swapChainExtent.height);
        const float screenX = ((ndcX * 0.5f) + 0.5f) * screenWidth + screenOffset.x;
        const float screenY = ((ndcY * 0.5f) + 0.5f) * screenHeight + screenOffset.y;

        DrawText(screenX, screenY, text, color, size);
        return true;
    }

    //Draws a UI line.
    void UIUtility::DrawLine(float startX, float startY, float endX, float endY, const Float4 &color, float thickness)
    {
        if (thickness <= 0.0f)
        {
            return;
        }

        if (m_vertices.size() + 4U > m_maxVertices || m_indices.size() + 6U > m_maxIndices)
        {
            return;
        }

        const float deltaX = endX - startX;
        const float deltaY = endY - startY;
        const float lineLength = std::sqrt((deltaX * deltaX) + (deltaY * deltaY));
        if (lineLength <= 0.0001f)
        {
            return;
        }

        const float halfThickness = thickness * 0.5f;
        const float offsetX = (-deltaY / lineLength) * halfThickness;
        const float offsetY = (deltaX / lineLength) * halfThickness;
        const uint32_t baseIndex = static_cast<uint32_t>(m_vertices.size());

        m_vertices.push_back(UIVertex{ { startX - offsetX, startY - offsetY }, color });
        m_vertices.push_back(UIVertex{ { startX + offsetX, startY + offsetY }, color });
        m_vertices.push_back(UIVertex{ { endX + offsetX, endY + offsetY }, color });
        m_vertices.push_back(UIVertex{ { endX - offsetX, endY - offsetY }, color });

        m_indices.push_back(baseIndex + 0U);
        m_indices.push_back(baseIndex + 1U);
        m_indices.push_back(baseIndex + 2U);
        m_indices.push_back(baseIndex + 2U);
        m_indices.push_back(baseIndex + 3U);
        m_indices.push_back(baseIndex + 0U);
    }

    //Draws a glass UI panel.
    void UIUtility::DrawGlassPanel(float x, float y, float width, float height)
    {
        const Float4 shadowColor{ 0.0f, 0.0f, 0.0f, 0.15f };
        const Float4 shadowSoftColor{ 0.0f, 0.0f, 0.0f, 0.07f };
        const Float4 panelColor = m_uiSetup.m_glassPanelColor;
        const Float4 panelTintColor = m_uiSetup.m_glassPanelTintColor;
        const Float4 panelHighlightColor{ 1.0f, 1.0f, 1.0f, 0.52f };
        const Float4 borderColor = m_uiSetup.m_glassBorderColor;

        AddRect(x + 12.0f, y + 14.0f, width, height, shadowSoftColor);
        AddRect(x + 6.0f, y + 8.0f, width, height, shadowColor);
        AddRect(x, y, width, height, panelColor);
        AddRect(x + 2.0f, y + 2.0f, width - 4.0f, height - 4.0f, panelTintColor);
        AddRect(x + 2.0f, y + 4.0f, width - 4.0f, 1.0f, panelHighlightColor);
        AddRect(x + 12.0f, y + 16.0f, width - 24.0f, 1.0f, { 1.0f, 1.0f, 1.0f, 0.16f });
        AddRectBorder(x, y, width, height, 2.0f, borderColor);
    }

    //Shows a stored popup window.
    void UIUtility::ShowPopup(const std::string &title, const std::string &message, const std::vector<std::string> &buttons, PopupCallback callback)
    {
        m_popupTitle = title;
        m_popupMessage = message;
        m_popupButtons = buttons;
        m_popupSliders.clear();
        m_popupCallback = callback;
        m_popupSliderButtonCallback = nullptr;
        m_popupSliderCallback = nullptr;
        m_popupVisible = true;
        m_popupPositionReady = false;
        m_popupDragging = false;
        m_popupActiveSliderIndex = -1;
        m_popupWidth = 560.0f;
        m_popupHeight = PopupContentHeight(m_popupWidth);
    }

    //Shows a stored popup window with sliders.
    void UIUtility::ShowPopup(
        const std::string &title,
        const std::string &message,
        const std::vector<std::string> &buttons,
        const std::vector<PopupSlider> &sliders,
        PopupSliderButtonCallback buttonCallback,
        PopupSliderCallback sliderCallback
    )
    {
        m_popupTitle = title;
        m_popupMessage = message;
        m_popupButtons = buttons;
        m_popupSliders = sliders;
        m_popupCallback = nullptr;
        m_popupSliderButtonCallback = buttonCallback;
        m_popupSliderCallback = sliderCallback;
        m_popupVisible = true;
        m_popupPositionReady = false;
        m_popupDragging = false;
        m_popupActiveSliderIndex = -1;
        m_popupWidth = 620.0f;
        m_popupHeight = PopupContentHeight(m_popupWidth);
    }

    //Updates the stored popup title and body text.
    void UIUtility::UpdatePopupText(const std::string &title, const std::string &message)
    {
        if (!m_popupVisible)
        {
            return;
        }

        m_popupTitle = title;
        m_popupMessage = message;
        m_popupHeight = PopupContentHeight(m_popupWidth);
    }

    //Hides the stored popup window.
    void UIUtility::HidePopup(void)
    {
        m_popupVisible = false;
        m_popupPositionReady = false;
        m_popupDragging = false;
        m_popupTitle.clear();
        m_popupMessage.clear();
        m_popupButtons.clear();
        m_popupSliders.clear();
        m_popupCallback = nullptr;
        m_popupSliderButtonCallback = nullptr;
        m_popupSliderCallback = nullptr;
        m_popupActiveSliderIndex = -1;
    }

    //Updates UI input state.
    void UIUtility::UpdateInput(const InputUtility &inputUtility)
    {
        m_inputUtilityPtr = &inputUtility;
    }

    //Draws stored UI windows.
    void UIUtility::DrawWindows(void)
    {
        if (!m_popupVisible || m_inputUtilityPtr == nullptr)
        {
            return;
        }

        const float screenWidth = static_cast<float>(m_swapChainExtent.width);
        const float screenHeight = static_cast<float>(m_swapChainExtent.height);
        const float popupWidth = std::max(180.0f, std::min(m_popupWidth, screenWidth - 40.0f));
        m_popupHeight = PopupContentHeight(popupWidth);
        const float popupHeight = std::max(160.0f, std::min(m_popupHeight, screenHeight - 40.0f));
        const float maximumPopupX = std::max(0.0f, screenWidth - popupWidth);
        const float maximumPopupY = std::max(0.0f, screenHeight - popupHeight);

        if (!m_popupPositionReady)
        {
            constexpr float popupMargin = 20.0f;
            m_popupPosition = {
                std::max(0.0f, screenWidth - popupWidth - popupMargin), popupMargin
            };
            m_popupPositionReady = true;
        }

        m_popupPosition.x = std::clamp(m_popupPosition.x, 0.0f, maximumPopupX);
        m_popupPosition.y = std::clamp(m_popupPosition.y, 0.0f, maximumPopupY);

        const int clickedButton = DrawPopup(m_popupPosition.x, m_popupPosition.y, popupWidth, popupHeight, m_popupTitle, m_popupMessage, m_popupButtons, m_popupSliders, *m_inputUtilityPtr);

        if (clickedButton >= 0)
        {
            PopupCallback callback = m_popupCallback;
            PopupSliderButtonCallback sliderButtonCallback = m_popupSliderButtonCallback;
            const std::vector<PopupSlider> sliders = m_popupSliders;
            HidePopup();

            if (sliderButtonCallback)
            {
                sliderButtonCallback(clickedButton, sliders);
            }
            else if (callback)
            {
                callback(clickedButton);
            }
        }
    }

    //Returns whether a stored popup is visible.
    bool UIUtility::PopupVisible(void) const
    {
        return m_popupVisible;
    }

    //Draws a popup window and returns clicked button index.
    int UIUtility::DrawPopup(
        float &x,
        float &y,
        float width,
        float height,
        const std::string &title,
        const std::string &message,
        const std::vector<std::string> &buttons,
        std::vector<PopupSlider> &sliders,
        const InputUtility &inputUtility
    )
    {
        const float titleHeight = 52.0f;
        const Float4 overlayColor{ 0.0f, 0.0f, 0.0f, 0.14f };
        const Float4 bodyColor{ 0.03f, 0.05f, 0.08f, 0.96f };
        const Float4 titleBarColor = m_uiSetup.m_glassHeaderColor;
        const Float4 textShadowColor = m_uiSetup.m_textShadowColor;
        const Float4 textColor = m_uiSetup.m_textColor;
        const Float4 accentColor = m_uiSetup.m_glassAccentColor;
        const Float4 borderColor = m_uiSetup.m_glassBorderColor;
        const Float4 buttonShadowColor{ 0.0f, 0.0f, 0.0f, 0.16f };
        const Float4 buttonColor = m_uiSetup.m_buttonColor;
        const Float4 buttonHoverColor = m_uiSetup.m_buttonHoverColor;
        const Float4 buttonPressedColor = m_uiSetup.m_buttonPressedColor;
        const float mouseX = static_cast<float>(inputUtility.MouseX());
        const float mouseY = static_cast<float>(inputUtility.MouseY());
        const bool wasPopupDragging = m_popupDragging;
        const bool wasSliderActive = m_popupActiveSliderIndex >= 0;

        if (MouseInRect(inputUtility, x, y, width, titleHeight) && inputUtility.MouseButtonPressed(0U))
        {
            m_popupDragging = true;
            m_popupDragOffset = { mouseX - x, mouseY - y };
        }

        if (!inputUtility.MouseButtonDown(0U))
        {
            m_popupDragging = false;
        }

        if (m_popupDragging)
        {
            const float maximumX = std::max(0.0f, static_cast<float>(m_swapChainExtent.width) - width);
            const float maximumY = std::max(0.0f, static_cast<float>(m_swapChainExtent.height) - height);
            x = std::clamp(mouseX - m_popupDragOffset.x, 0.0f, maximumX);
            y = std::clamp(mouseY - m_popupDragOffset.y, 0.0f, maximumY);
        }

        AddRect(0.0f, 0.0f, static_cast<float>(m_swapChainExtent.width), static_cast<float>(m_swapChainExtent.height), overlayColor);
        AddRect(x, y, width, height, bodyColor);
        AddRect(x, y, width, titleHeight, titleBarColor);
        AddRect(x, y + titleHeight, width, 2.0f, accentColor);
        AddRectBorder(x, y, width, height, 2.0f, borderColor);
        DrawText(x + 21.0f, y + 16.0f, title, textShadowColor, 2.6f);
        DrawText(x + 20.0f, y + 15.0f, title, textColor, 2.6f);
        DrawWrappedText(x + 25.0f, y + 77.0f, width - 48.0f, message, textShadowColor, 2.0f);
        DrawWrappedText(x + 24.0f, y + 76.0f, width - 48.0f, message, textColor, 2.0f);
        DrawPopupSliders(x, y, width, sliders, inputUtility);

        int clickedButton = -1;
        const float buttonHeight = 34.0f;
        const float buttonY = y + height - 52.0f;
        const float buttonTextSize = 2.0f;
        const float buttonGap = 12.0f;
        std::vector<float> buttonWidths(buttons.size());
        float totalButtonWidth = 0.0f;

        for (std::size_t i = 0; i < buttons.size(); ++i)
        {
            buttonWidths[i] = std::max(84.0f, TextWidth(buttons[i], buttonTextSize) + 28.0f);
            totalButtonWidth += buttonWidths[i];
        }

        if (!buttons.empty())
        {
            totalButtonWidth += buttonGap * static_cast<float>(buttons.size() - 1U);
        }

        float buttonX = x + width - 22.0f - totalButtonWidth;
        for (std::size_t i = 0; i < buttons.size(); ++i)
        {
            const bool hovered = MouseInRect(inputUtility, buttonX, buttonY, buttonWidths[i], buttonHeight);
            const bool pressed = hovered && inputUtility.MouseButtonDown(0U);
            const Float4 drawColor = pressed ? buttonPressedColor : (hovered ? buttonHoverColor : buttonColor);
            const float textWidth = TextWidth(buttons[i], buttonTextSize);
            const float textHeight = LineHeight(buttonTextSize);
            const float textX = buttonX + ((buttonWidths[i] - textWidth) * 0.5f);
            const float textY = buttonY + ((buttonHeight - textHeight) * 0.5f);

            AddRect(buttonX + 2.0f, buttonY + 3.0f, buttonWidths[i], buttonHeight, buttonShadowColor);
            AddRect(buttonX, buttonY, buttonWidths[i], buttonHeight, drawColor);
            AddRectBorder(buttonX, buttonY, buttonWidths[i], buttonHeight, 1.0f, borderColor);
            DrawText(textX + 1.0f, textY + 1.0f, buttons[i], textShadowColor, buttonTextSize);
            DrawText(textX, textY, buttons[i], textColor, buttonTextSize);

            if (hovered && !wasPopupDragging && !wasSliderActive && inputUtility.MouseButtonReleased(0U))
            {
                clickedButton = static_cast<int>(i);
            }

            buttonX += buttonWidths[i] + buttonGap;
        }

        return clickedButton;
    }

    //Draws popup sliders.
    void UIUtility::DrawPopupSliders(float x, float y, float width, std::vector<PopupSlider> &sliders, const InputUtility &inputUtility)
    {
        if (sliders.empty())
        {
            return;
        }

        if (!inputUtility.MouseButtonDown(0U))
        {
            m_popupActiveSliderIndex = -1;
        }

        const Float4 textColor = m_uiSetup.m_textColor;
        const Float4 textShadowColor = m_uiSetup.m_textShadowColor;
        const Float4 trackColor{ 0.08f, 0.14f, 0.21f, 0.74f };
        const Float4 trackFillColor = m_uiSetup.m_glassAccentColor;
        const Float4 thumbColor = m_uiSetup.m_buttonHoverColor;
        const Float4 thumbActiveColor = m_uiSetup.m_buttonPressedColor;
        const Float4 borderColor = m_uiSetup.m_glassBorderColor;
        const float sliderStartY = y + 128.0f;
        const float rowHeight = 58.0f;
        const float trackX = x + 28.0f;
        const float trackWidth = width - 56.0f;
        const float trackHeight = 8.0f;
        const float thumbWidth = 14.0f;
        const float thumbHeight = 24.0f;
        const float labelTextSize = 1.8f;
        const float valueTextSize = 1.7f;
        const float mouseX = static_cast<float>(inputUtility.MouseX());

        for (std::size_t sliderIndex = 0U; sliderIndex < sliders.size(); ++sliderIndex)
        {
            PopupSlider &slider = sliders[sliderIndex];
            const float minimum = std::min(slider.m_minimum, slider.m_maximum);
            const float maximum = std::max(slider.m_minimum, slider.m_maximum);
            const float range = maximum - minimum;
            slider.m_value = std::clamp(slider.m_value, minimum, maximum);

            const float rowY = sliderStartY + (static_cast<float>(sliderIndex) * rowHeight);
            const float trackY = rowY + 30.0f;
            const bool hasRange = range > 0.000001f;
            const float percent = hasRange ? std::clamp((slider.m_value - minimum) / range, 0.0f, 1.0f) : 0.0f;
            const float fillWidth = trackWidth * percent;
            const float thumbX = trackX + fillWidth - (thumbWidth * 0.5f);
            const bool hovered = MouseInRect(inputUtility, trackX - 8.0f, trackY - 10.0f, trackWidth + 16.0f, thumbHeight);

            if (hovered && inputUtility.MouseButtonPressed(0U))
            {
                m_popupActiveSliderIndex = static_cast<int>(sliderIndex);
            }

            if (m_popupActiveSliderIndex == static_cast<int>(sliderIndex) && inputUtility.MouseButtonDown(0U) && hasRange)
            {
                const float newPercent = std::clamp((mouseX - trackX) / trackWidth, 0.0f, 1.0f);
                const float newValue = minimum + (newPercent * range);
                const float changedAmount = std::fabs(newValue - slider.m_value);
                if (changedAmount > std::max(0.00001f, range * 0.0001f))
                {
                    slider.m_value = newValue;
                    if (m_popupSliderCallback)
                    {
                        m_popupSliderCallback(static_cast<int>(sliderIndex), slider.m_value);
                    }
                }
            }

            const std::string valueText = FormatSliderValue(slider.m_value);
            const float valueWidth = TextWidth(valueText, valueTextSize);

            DrawText(trackX + 1.0f, rowY + 1.0f, slider.m_label, textShadowColor, labelTextSize);
            DrawText(trackX, rowY, slider.m_label, textColor, labelTextSize);
            DrawText(trackX + trackWidth - valueWidth + 1.0f, rowY + 1.0f, valueText, textShadowColor, valueTextSize);
            DrawText(trackX + trackWidth - valueWidth, rowY, valueText, textColor, valueTextSize);

            AddRect(trackX, trackY, trackWidth, trackHeight, trackColor);
            AddRect(trackX, trackY, fillWidth, trackHeight, trackFillColor);
            AddRectBorder(trackX, trackY, trackWidth, trackHeight, 1.0f, borderColor);
            AddRect(thumbX, trackY - ((thumbHeight - trackHeight) * 0.5f), thumbWidth, thumbHeight, m_popupActiveSliderIndex == static_cast<int>(sliderIndex) ? thumbActiveColor : thumbColor);
            AddRectBorder(thumbX, trackY - ((thumbHeight - trackHeight) * 0.5f), thumbWidth, thumbHeight, 1.0f, borderColor);
        }
    }

    //Returns a formatted slider value.
    std::string UIUtility::FormatSliderValue(float value) const
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2) << value;
        return stream.str();
    }

    //Draws UI geometry to a command buffer.
    void UIUtility::Show(VkCommandBuffer commandBufferPtrGPU)
    {
        if (m_pipelinePtrGPU == VK_NULL_HANDLE || m_currentFrame >= m_vertexBuffersPtrGPU.size() || m_currentFrame >= m_indexBuffersPtrGPU.size() ||
            m_vertexBuffersPtrGPU[m_currentFrame] == VK_NULL_HANDLE || m_indexBuffersPtrGPU[m_currentFrame] == VK_NULL_HANDLE || m_vertices.empty() || m_indices.empty())
        {
            return;
        }

        std::memcpy(m_vertexMappedPtr[m_currentFrame], m_vertices.data(), m_vertices.size() * sizeof(UIVertex));
        std::memcpy(m_indexMappedPtr[m_currentFrame], m_indices.data(), m_indices.size() * sizeof(uint32_t));

        vkCmdBindPipeline(commandBufferPtrGPU, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelinePtrGPU);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(m_swapChainExtent.width);
        viewport.height = static_cast<float>(m_swapChainExtent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(commandBufferPtrGPU, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.offset = { 0, 0 };
        scissor.extent = m_swapChainExtent;
        vkCmdSetScissor(commandBufferPtrGPU, 0, 1, &scissor);

        UIShaderConstants constants{};
        constants.m_screenSize = {
            static_cast<float>(m_swapChainExtent.width), static_cast<float>(m_swapChainExtent.height)
        };

        vkCmdPushConstants(commandBufferPtrGPU, m_pipelineLayoutPtrGPU, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(UIShaderConstants), &constants);

        const VkBuffer vertexBuffersPtrGPU[] = { m_vertexBuffersPtrGPU[m_currentFrame] };
        const VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(commandBufferPtrGPU, 0, 1, vertexBuffersPtrGPU, offsets);
        vkCmdBindIndexBuffer(commandBufferPtrGPU, m_indexBuffersPtrGPU[m_currentFrame], 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(commandBufferPtrGPU, static_cast<uint32_t>(m_indices.size()), 1, 0, 0, 0);
    }

    //Releases only the graphics pipeline.
    void UIUtility::CleanPipeline(void)
    {
        if (m_devicePtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        if (m_pipelinePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(m_devicePtrGPU, m_pipelinePtrGPU, nullptr);
            m_pipelinePtrGPU = VK_NULL_HANDLE;
        }
    }

    //Recreates the pipeline after swapchain changes.
    void UIUtility::RefreshPipeline(VkRenderPass renderPassPtrGPU, VkExtent2D swapChainExtent)
    {
        if (m_devicePtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        m_renderPassPtrGPU = renderPassPtrGPU;
        m_swapChainExtent = swapChainExtent;
        CleanPipeline();
        CreatePipeline();
    }

    //Releases all UI GPU resources.
    void UIUtility::Clean(void)
    {
        HidePopup();
        m_inputUtilityPtr = nullptr;

        if (m_devicePtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        CleanPipeline();

        if (m_pipelineLayoutPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(m_devicePtrGPU, m_pipelineLayoutPtrGPU, nullptr);
            m_pipelineLayoutPtrGPU = VK_NULL_HANDLE;
        }

        for (std::size_t i = 0; i < m_vertexBufferMemoriesPtrGPU.size(); ++i)
        {
            if (m_vertexBufferMemoriesPtrGPU[i] != VK_NULL_HANDLE && m_vertexMappedPtr[i] != nullptr)
            {
                vkUnmapMemory(m_devicePtrGPU, m_vertexBufferMemoriesPtrGPU[i]);
                m_vertexMappedPtr[i] = nullptr;
            }
        }

        for (std::size_t i = 0; i < m_indexBufferMemoriesPtrGPU.size(); ++i)
        {
            if (m_indexBufferMemoriesPtrGPU[i] != VK_NULL_HANDLE && m_indexMappedPtr[i] != nullptr)
            {
                vkUnmapMemory(m_devicePtrGPU, m_indexBufferMemoriesPtrGPU[i]);
                m_indexMappedPtr[i] = nullptr;
            }
        }

        for (VkBuffer &vertexBufferPtrGPU : m_vertexBuffersPtrGPU)
        {
            if (vertexBufferPtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyBuffer(m_devicePtrGPU, vertexBufferPtrGPU, nullptr);
                vertexBufferPtrGPU = VK_NULL_HANDLE;
            }
        }

        for (VkDeviceMemory &vertexBufferMemoryPtrGPU : m_vertexBufferMemoriesPtrGPU)
        {
            if (vertexBufferMemoryPtrGPU != VK_NULL_HANDLE)
            {
                vkFreeMemory(m_devicePtrGPU, vertexBufferMemoryPtrGPU, nullptr);
                vertexBufferMemoryPtrGPU = VK_NULL_HANDLE;
            }
        }

        for (VkBuffer &indexBufferPtrGPU : m_indexBuffersPtrGPU)
        {
            if (indexBufferPtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyBuffer(m_devicePtrGPU, indexBufferPtrGPU, nullptr);
                indexBufferPtrGPU = VK_NULL_HANDLE;
            }
        }

        for (VkDeviceMemory &indexBufferMemoryPtrGPU : m_indexBufferMemoriesPtrGPU)
        {
            if (indexBufferMemoryPtrGPU != VK_NULL_HANDLE)
            {
                vkFreeMemory(m_devicePtrGPU, indexBufferMemoryPtrGPU, nullptr);
                indexBufferMemoryPtrGPU = VK_NULL_HANDLE;
            }
        }

        m_vertexBuffersPtrGPU.clear();
        m_vertexBufferMemoriesPtrGPU.clear();
        m_indexBuffersPtrGPU.clear();
        m_indexBufferMemoriesPtrGPU.clear();
        m_vertexMappedPtr.clear();
        m_indexMappedPtr.clear();
        m_vertices.clear();
        m_indices.clear();
        m_fontGlyphs.clear();
        m_devicePtrGPU = VK_NULL_HANDLE;
        m_physicalDevicePtrGPU = VK_NULL_HANDLE;
        m_renderPassPtrGPU = VK_NULL_HANDLE;
        m_frameCount = 0U;
        m_currentFrame = 0U;
    }

    //Creates the UI graphics pipeline.
    void UIUtility::CreatePipeline(void)
    {
        const auto vertexShaderCode = ReadShaderFile("shaders/ui.vert.spv");
        const auto fragmentShaderCode = ReadShaderFile("shaders/ui.frag.spv");

        const VkShaderModule vertexShaderModulePtrGPU = CreateShaderModule(vertexShaderCode);
        const VkShaderModule fragmentShaderModulePtrGPU = CreateShaderModule(fragmentShaderCode);

        VkPipelineShaderStageCreateInfo vertexShaderStageInfo{};
        vertexShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertexShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderStageInfo.module = vertexShaderModulePtrGPU;
        vertexShaderStageInfo.pName = "main";

        VkPipelineShaderStageCreateInfo fragmentShaderStageInfo{};
        fragmentShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        fragmentShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderStageInfo.module = fragmentShaderModulePtrGPU;
        fragmentShaderStageInfo.pName = "main";

        const VkPipelineShaderStageCreateInfo shaderStages[] = {
            vertexShaderStageInfo, fragmentShaderStageInfo
        };

        const auto bindingDescription = UIVertex::BindingDescription();
        const auto attributeDescriptions = UIVertex::AttributeDescriptions();

        VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInputInfo.vertexBindingDescriptionCount = 1;
        vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
        vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
        vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        inputAssembly.primitiveRestartEnable = VK_FALSE;

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_FALSE;
        rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.depthBiasEnable = VK_FALSE;

        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.sampleShadingEnable = VK_FALSE;
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_FALSE;
        depthStencil.depthWriteEnable = VK_FALSE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_ALWAYS;

        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        colorBlendAttachment.blendEnable = VK_TRUE;
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

        VkPipelineColorBlendStateCreateInfo colorBlending{};
        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.logicOpEnable = VK_FALSE;
        colorBlending.attachmentCount = 1;
        colorBlending.pAttachments = &colorBlendAttachment;

        const std::array<VkDynamicState, 2> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR
        };

        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        if (m_pipelineLayoutPtrGPU == VK_NULL_HANDLE)
        {
            VkPushConstantRange pushConstantRange{};
            pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
            pushConstantRange.offset = 0;
            pushConstantRange.size = sizeof(UIShaderConstants);

            VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
            pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            pipelineLayoutInfo.setLayoutCount = 0;
            pipelineLayoutInfo.pushConstantRangeCount = 1;
            pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

            if (vkCreatePipelineLayout(m_devicePtrGPU, &pipelineLayoutInfo, nullptr, &m_pipelineLayoutPtrGPU) != VK_SUCCESS)
            {
                vkDestroyShaderModule(m_devicePtrGPU, fragmentShaderModulePtrGPU, nullptr);
                vkDestroyShaderModule(m_devicePtrGPU, vertexShaderModulePtrGPU, nullptr);
                throw std::runtime_error("Failed to create UI pipeline layout.");
            }
        }

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 2;
        pipelineInfo.pStages = shaderStages;
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlending;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = m_pipelineLayoutPtrGPU;
        pipelineInfo.renderPass = m_renderPassPtrGPU;
        pipelineInfo.subpass = 0;

        if (vkCreateGraphicsPipelines(m_devicePtrGPU, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_pipelinePtrGPU) != VK_SUCCESS)
        {
            vkDestroyShaderModule(m_devicePtrGPU, fragmentShaderModulePtrGPU, nullptr);
            vkDestroyShaderModule(m_devicePtrGPU, vertexShaderModulePtrGPU, nullptr);
            throw std::runtime_error("Failed to create UI graphics pipeline.");
        }

        vkDestroyShaderModule(m_devicePtrGPU, fragmentShaderModulePtrGPU, nullptr);
        vkDestroyShaderModule(m_devicePtrGPU, vertexShaderModulePtrGPU, nullptr);
    }

    //Creates the UI vertex and index buffers.
    void UIUtility::CreateBuffers(void)
    {
        m_vertexBuffersPtrGPU.resize(m_frameCount, VK_NULL_HANDLE);
        m_vertexBufferMemoriesPtrGPU.resize(m_frameCount, VK_NULL_HANDLE);
        m_indexBuffersPtrGPU.resize(m_frameCount, VK_NULL_HANDLE);
        m_indexBufferMemoriesPtrGPU.resize(m_frameCount, VK_NULL_HANDLE);
        m_vertexMappedPtr.resize(m_frameCount, nullptr);
        m_indexMappedPtr.resize(m_frameCount, nullptr);

        for (std::size_t i = 0; i < m_frameCount; ++i)
        {
            CreateBuffer(
                static_cast<VkDeviceSize>(sizeof(UIVertex) * m_maxVertices),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                m_vertexBuffersPtrGPU[i],
                m_vertexBufferMemoriesPtrGPU[i]
            );

            CreateBuffer(
                static_cast<VkDeviceSize>(sizeof(uint32_t) * m_maxIndices),
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                m_indexBuffersPtrGPU[i],
                m_indexBufferMemoriesPtrGPU[i]
            );

            if (vkMapMemory(m_devicePtrGPU, m_vertexBufferMemoriesPtrGPU[i], 0, VK_WHOLE_SIZE, 0, &m_vertexMappedPtr[i]) != VK_SUCCESS)
            {
                throw std::runtime_error("Failed to map UI vertex buffer.");
            }

            if (vkMapMemory(m_devicePtrGPU, m_indexBufferMemoriesPtrGPU[i], 0, VK_WHOLE_SIZE, 0, &m_indexMappedPtr[i]) != VK_SUCCESS)
            {
                throw std::runtime_error("Failed to map UI index buffer.");
            }
        }
    }

    //Creates a Vulkan buffer and memory.
    void UIUtility::CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &bufferPtrGPU, VkDeviceMemory &bufferMemoryPtrGPU) const
    {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateBuffer(m_devicePtrGPU, &bufferInfo, nullptr, &bufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create UI buffer.");
        }

        VkMemoryRequirements memoryRequirements{};
        vkGetBufferMemoryRequirements(m_devicePtrGPU, bufferPtrGPU, &memoryRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(m_devicePtrGPU, &allocInfo, nullptr, &bufferMemoryPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate UI buffer memory.");
        }

        vkBindBufferMemory(m_devicePtrGPU, bufferPtrGPU, bufferMemoryPtrGPU, 0);
    }

    //Creates a Vulkan shader module.
    VkShaderModule UIUtility::CreateShaderModule(const std::vector<char> &code) const
    {
        if (code.empty() || (code.size() % sizeof(uint32_t)) != 0U)
        {
            throw std::runtime_error("Invalid UI shader code.");
        }

        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());

        VkShaderModule shaderModulePtrGPU = VK_NULL_HANDLE;
        if (vkCreateShaderModule(m_devicePtrGPU, &createInfo, nullptr, &shaderModulePtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create UI shader module.");
        }

        return shaderModulePtrGPU;
    }

    //Finds a matching memory type.
    uint32_t UIUtility::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const
    {
        VkPhysicalDeviceMemoryProperties memoryProperties{};
        vkGetPhysicalDeviceMemoryProperties(m_physicalDevicePtrGPU, &memoryProperties);

        for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
        {
            if ((typeFilter & (1U << i)) && (memoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
            {
                return i;
            }
        }

        throw std::runtime_error("Failed to find a suitable UI memory type.");
    }

    //Loads a compiled shader file.
    std::vector<char> UIUtility::ReadShaderFile(const std::string &relativePath) const
    {
        std::vector<std::filesystem::path> candidates;

        wchar_t modulePath[MAX_PATH]{};
        const DWORD modulePathLength = GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
        if (modulePathLength > 0)
        {
            candidates.push_back(std::filesystem::path(modulePath).parent_path() / relativePath);
        }

        candidates.push_back(PathUtility::ShaderOutputFolder() / std::filesystem::path(relativePath).filename());
        candidates.push_back(std::filesystem::current_path() / relativePath);
        candidates.push_back(std::filesystem::path(relativePath));

        for (const auto &candidate : candidates)
        {
            std::ifstream file(candidate, std::ios::ate | std::ios::binary);
            if (!file.is_open())
            {
                continue;
            }

            const std::streampos endPosition = file.tellg();
            if (endPosition == std::streampos(-1))
            {
                continue;
            }

            const std::size_t fileSize = static_cast<std::size_t>(endPosition);
            if (fileSize == 0)
            {
                return {};
            }

            std::vector<char> buffer(fileSize);
            file.seekg(0);
            file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            file.close();
            return buffer;
        }

        throw std::runtime_error("Failed to open UI shader file.");
    }

    //Finds or creates a cached font glyph.
    const UIUtility::FontGlyph &UIUtility::FindGlyph(char character, float size)
    {
        const int pixelHeight = std::max(8, static_cast<int>(std::lround(size * s_fontPixelScale)));
        const unsigned char characterCode = static_cast<unsigned char>(character);
        const char safeCharacter = (characterCode >= 32U && characterCode <= 126U) ? character : '?';

        for (const FontGlyph &glyph : m_fontGlyphs)
        {
            if (glyph.m_character == safeCharacter && glyph.m_pixelHeight == pixelHeight)
            {
                return glyph;
            }
        }

        m_fontGlyphs.push_back(BuildGlyph(safeCharacter, pixelHeight));
        return m_fontGlyphs.back();
    }

    //Builds one font glyph using Windows GDI.
    UIUtility::FontGlyph UIUtility::BuildGlyph(char character, int pixelHeight) const
    {
        FontGlyph glyph{};
        glyph.m_character = character;
        glyph.m_pixelHeight = pixelHeight;

        HDC dcPtr = CreateCompatibleDC(nullptr);
        if (dcPtr == nullptr)
        {
            throw std::runtime_error("Failed to create UI font device context.");
        }

        HFONT fontPtr = CreateFontW(
            -pixelHeight,
            0,
            0,
            0,
            FW_SEMIBOLD,
            FALSE,
            FALSE,
            FALSE,
            DEFAULT_CHARSET,
            OUT_DEFAULT_PRECIS,
            CLIP_DEFAULT_PRECIS,
            CLEARTYPE_NATURAL_QUALITY,
            DEFAULT_PITCH | FF_DONTCARE,
            L"Segoe UI"
        );

        if (fontPtr == nullptr)
        {
            DeleteDC(dcPtr);
            throw std::runtime_error("Failed to create UI font.");
        }

        HGDIOBJ oldFontPtr = SelectObject(dcPtr, fontPtr);
        SetBkMode(dcPtr, TRANSPARENT);
        SetTextColor(dcPtr, RGB(255, 255, 255));
        SetBkColor(dcPtr, RGB(0, 0, 0));

        const wchar_t wideCharacter = static_cast<wchar_t>(static_cast<unsigned char>(character));
        TEXTMETRICW textMetric{};
        SIZE textSize{};

        if (GetTextMetricsW(dcPtr, &textMetric) == 0 || GetTextExtentPoint32W(dcPtr, &wideCharacter, 1, &textSize) == 0)
        {
            if (oldFontPtr != nullptr)
            {
                SelectObject(dcPtr, oldFontPtr);
            }

            DeleteObject(fontPtr);
            DeleteDC(dcPtr);
            throw std::runtime_error("Failed to measure UI font glyph.");
        }

        glyph.m_advance = std::max(1, static_cast<int>(textSize.cx));
        const int bitmapWidth = std::max(1, static_cast<int>(textSize.cx) + (s_glyphPadding * 2));
        const int bitmapHeight = std::max(1, static_cast<int>(textMetric.tmHeight) + (s_glyphPadding * 2));

        BITMAPINFO bitmapInfo{};
        bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bitmapInfo.bmiHeader.biWidth = bitmapWidth;
        bitmapInfo.bmiHeader.biHeight = -bitmapHeight;
        bitmapInfo.bmiHeader.biPlanes = 1;
        bitmapInfo.bmiHeader.biBitCount = 32;
        bitmapInfo.bmiHeader.biCompression = BI_RGB;

        void *bitsPtr = nullptr;
        HBITMAP bitmapPtr = CreateDIBSection(dcPtr, &bitmapInfo, DIB_RGB_COLORS, &bitsPtr, nullptr, 0);
        if (bitmapPtr == nullptr || bitsPtr == nullptr)
        {
            if (oldFontPtr != nullptr)
            {
                SelectObject(dcPtr, oldFontPtr);
            }

            DeleteObject(fontPtr);
            DeleteDC(dcPtr);
            throw std::runtime_error("Failed to create UI font bitmap.");
        }

        HGDIOBJ oldBitmapPtr = SelectObject(dcPtr, bitmapPtr);
        std::memset(bitsPtr, 0, static_cast<std::size_t>(bitmapWidth) * static_cast<std::size_t>(bitmapHeight) * 4U);
        TextOutW(dcPtr, s_glyphPadding, s_glyphPadding, &wideCharacter, 1);

        glyph.m_width = bitmapWidth;
        glyph.m_height = bitmapHeight;
        glyph.m_alpha.resize(static_cast<std::size_t>(bitmapWidth) * static_cast<std::size_t>(bitmapHeight));

        const uint8_t *pixelBytesPtr = static_cast<const uint8_t *>(bitsPtr);
        for (int row = 0; row < bitmapHeight; ++row)
        {
            for (int column = 0; column < bitmapWidth; ++column)
            {
                const std::size_t pixelIndex = static_cast<std::size_t>((row * bitmapWidth) + column);
                const std::size_t byteIndex = pixelIndex * 4U;
                const uint8_t blue = pixelBytesPtr[byteIndex + 0U];
                const uint8_t green = pixelBytesPtr[byteIndex + 1U];
                const uint8_t red = pixelBytesPtr[byteIndex + 2U];
                glyph.m_alpha[pixelIndex] = std::max(red, std::max(green, blue));
            }
        }

        if (oldBitmapPtr != nullptr)
        {
            SelectObject(dcPtr, oldBitmapPtr);
        }

        DeleteObject(bitmapPtr);

        if (oldFontPtr != nullptr)
        {
            SelectObject(dcPtr, oldFontPtr);
        }

        DeleteObject(fontPtr);
        DeleteDC(dcPtr);
        return glyph;
    }

    //Draws one cached font glyph.
    void UIUtility::DrawGlyph(float x, float y, const FontGlyph &glyph, const Float4 &color)
    {
        if (glyph.m_width <= 0 || glyph.m_height <= 0)
        {
            return;
        }

        for (int row = 0; row < glyph.m_height; ++row)
        {
            int runStart = -1;
            uint8_t runAlpha = 0U;

            for (int column = 0; column <= glyph.m_width; ++column)
            {
                uint8_t alpha = 0U;
                if (column < glyph.m_width)
                {
                    const std::size_t pixelIndex = static_cast<std::size_t>((row * glyph.m_width) + column);
                    alpha = QuantizeAlpha(glyph.m_alpha[pixelIndex]);
                }

                if (alpha == runAlpha)
                {
                    continue;
                }

                if (runAlpha > 0U && runStart >= 0)
                {
                    Float4 drawColor = color;
                    drawColor.w *= static_cast<float>(runAlpha) / 255.0f;
                    AddRect(x + static_cast<float>(runStart), y + static_cast<float>(row), static_cast<float>(column - runStart), 1.0f, drawColor);
                }

                runAlpha = alpha;
                runStart = (alpha > 0U) ? column : -1;
            }
        }
    }

    //Returns the rendered text width.
    float UIUtility::TextWidth(const std::string &text, float size)
    {
        float currentWidth = 0.0f;
        float maximumWidth = 0.0f;

        for (char character : text)
        {
            if (character == '\r')
            {
                continue;
            }

            if (character == '\n')
            {
                maximumWidth = std::max(maximumWidth, currentWidth);
                currentWidth = 0.0f;
                continue;
            }

            if (character == '\t')
            {
                currentWidth += static_cast<float>(FindGlyph(' ', size).m_advance) * 4.0f;
                continue;
            }

            currentWidth += static_cast<float>(FindGlyph(character, size).m_advance);
        }

        return std::max(maximumWidth, currentWidth);
    }

    //Returns the rendered line height.
    float UIUtility::LineHeight(float size)
    {
        return static_cast<float>(FindGlyph('M', size).m_height + 3);
    }

    //Returns the wrapped text height.
    float UIUtility::WrappedTextHeight(const std::string &text, float width, float size)
    {
        std::string word;
        float cursorX = 0.0f;
        const float safeWidth = std::max(1.0f, width);
        const float spaceWidth = TextWidth(" ", size);
        const float lineHeight = LineHeight(size);
        int lineCount = 1;

        const auto appendWord = [&](const std::string &drawText)
        {
            if (drawText.empty())
            {
                return;
            }

            const float wordWidth = TextWidth(drawText, size);
            if (cursorX > 0.0f && cursorX + wordWidth > safeWidth)
            {
                cursorX = 0.0f;
                ++lineCount;
            }

            cursorX += wordWidth + spaceWidth;
        };

        for (char character : text)
        {
            if (character == '\n')
            {
                appendWord(word);
                word.clear();
                cursorX = 0.0f;
                ++lineCount;
                continue;
            }

            if (std::isspace(static_cast<unsigned char>(character)) != 0)
            {
                appendWord(word);
                word.clear();
                continue;
            }

            word.push_back(character);
        }

        appendWord(word);
        return static_cast<float>(lineCount) * lineHeight;
    }

    //Returns the popup height needed for the current content.
    float UIUtility::PopupContentHeight(float width)
    {
        const float messageWidth = std::max(1.0f, width - 48.0f);
        const float messageHeight = WrappedTextHeight(m_popupMessage, messageWidth, 2.0f);
        const float buttonReserve = m_popupButtons.empty() ? 42.0f : 78.0f;
        const float sliderReserve = static_cast<float>(m_popupSliders.size()) * 58.0f;
        const float minimumHeight = m_popupSliders.empty() ? 190.0f : 280.0f;

        return std::max(minimumHeight, 104.0f + messageHeight + buttonReserve + sliderReserve);
    }

    //Adds a filled rectangle.
    void UIUtility::AddRect(float x, float y, float width, float height, const Float4 &color)
    {
        if (width <= 0.0f || height <= 0.0f)
        {
            return;
        }

        if (m_vertices.size() + 4U > m_maxVertices || m_indices.size() + 6U > m_maxIndices)
        {
            return;
        }

        const uint32_t baseIndex = static_cast<uint32_t>(m_vertices.size());
        m_vertices.push_back(UIVertex{ { x, y }, color });
        m_vertices.push_back(UIVertex{ { x + width, y }, color });
        m_vertices.push_back(UIVertex{ { x + width, y + height }, color });
        m_vertices.push_back(UIVertex{ { x, y + height }, color });

        m_indices.push_back(baseIndex + 0U);
        m_indices.push_back(baseIndex + 1U);
        m_indices.push_back(baseIndex + 2U);
        m_indices.push_back(baseIndex + 2U);
        m_indices.push_back(baseIndex + 3U);
        m_indices.push_back(baseIndex + 0U);
    }

    //Adds a rectangle border.
    void UIUtility::AddRectBorder(float x, float y, float width, float height, float thickness, const Float4 &color)
    {
        AddRect(x, y, width, thickness, color);
        AddRect(x, y + height - thickness, width, thickness, color);
        AddRect(x, y, thickness, height, color);
        AddRect(x + width - thickness, y, thickness, height, color);
    }

    //Draws wrapped UI text.
    void UIUtility::DrawWrappedText(float x, float y, float width, const std::string &text, const Float4 &color, float size)
    {
        std::string word;
        float cursorX = x;
        float cursorY = y;
        const float spaceWidth = TextWidth(" ", size);
        const float lineHeight = LineHeight(size);

        const auto drawWord = [&](const std::string &drawText)
        {
            if (drawText.empty())
            {
                return;
            }

            const float wordWidth = TextWidth(drawText, size);
            if (cursorX > x && cursorX + wordWidth > x + width)
            {
                cursorX = x;
                cursorY += lineHeight;
            }

            DrawText(cursorX, cursorY, drawText, color, size);
            cursorX += wordWidth + spaceWidth;
        };

        for (char character : text)
        {
            if (character == '\n')
            {
                drawWord(word);
                word.clear();
                cursorX = x;
                cursorY += lineHeight;
                continue;
            }

            if (std::isspace(static_cast<unsigned char>(character)) != 0)
            {
                drawWord(word);
                word.clear();
                continue;
            }

            word.push_back(character);
        }

        drawWord(word);
    }

    //Returns whether the mouse is inside a rectangle.
    bool UIUtility::MouseInRect(const InputUtility &inputUtility, float x, float y, float width, float height) const
    {
        const float mouseX = static_cast<float>(inputUtility.MouseX());
        const float mouseY = static_cast<float>(inputUtility.MouseY());
        return mouseX >= x && mouseX <= x + width && mouseY >= y && mouseY <= y + height;
    }

    //Returns an alpha value suitable for smooth glyph drawing.
    uint8_t UIUtility::QuantizeAlpha(uint8_t alpha)
    {
        if (alpha < s_minimumGlyphAlpha)
        {
            return 0U;
        }

        return alpha;
    }
}

