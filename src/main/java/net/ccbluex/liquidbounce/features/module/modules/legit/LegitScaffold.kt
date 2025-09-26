package net.ccbluex.liquidbounce.features.module.modules.legit

import net.ccbluex.liquidbounce.event.EventTarget
import net.ccbluex.liquidbounce.event.Render2DEvent
import net.ccbluex.liquidbounce.features.module.Module
import net.ccbluex.liquidbounce.features.module.ModuleCategory
import net.ccbluex.liquidbounce.features.module.ModuleInfo
import net.ccbluex.liquidbounce.features.value.BoolValue
import net.ccbluex.liquidbounce.features.value.FloatValue
import net.ccbluex.liquidbounce.features.value.IntegerValue
import net.ccbluex.liquidbounce.utils.MovementUtils
import net.ccbluex.liquidbounce.utils.block.BlockUtils
import net.ccbluex.liquidbounce.utils.render.RenderUtils
import net.ccbluex.liquidbounce.utils.timer.MSTimer
import net.minecraft.block.BlockAir
import net.minecraft.client.gui.FontRenderer
import net.minecraft.client.settings.KeyBinding
import net.minecraft.item.ItemBlock
import net.minecraft.util.BlockPos
import org.lwjgl.input.Keyboard
import kotlin.random.Random

@ModuleInfo(name = "LegitScaffold", category = ModuleCategory.LEGIT)
class LegitScaffold : Module() {
    // Delay settings
    private val minDelay = IntegerValue("MinDelay", 100, 0, 500)
    private val maxDelay = IntegerValue("MaxDelay", 200, 0, 500)

    // Angle check settings
    private val pitchCheck = BoolValue("PitchCheck", true)
    private val pitch = FloatValue("Pitch", 45f, 0f, 90f).displayable { pitchCheck.get() }

    // Activation conditions
    private val onlySPressed = BoolValue("OnlySPressed", false)
    private val onlySneak = BoolValue("OnlySneak", false)
    private val showBlockCount = BoolValue("ShowBlockCount", false)

    // State variables
    private var lastSneakTime = -1L
    private val timer = MSTimer()

    init {
        correctValueRange() // Initial range check
    }

    // Ensure min <= max (called manually when values change)
    private fun correctValueRange() {
        if (minDelay.get() > maxDelay.get()) {
            maxDelay.set(minDelay.get())
        }
    }

    // Reset state when disabled
    override fun onDisable() {
        lastSneakTime = -1
        setSneak(Keyboard.isKeyDown(mc.gameSettings.keyBindSneak.keyCode))
    }

    @EventTarget
    fun onRender(event: Render2DEvent) {
        if (mc.currentScreen != null || mc.thePlayer == null) return

        // Update value range (manual check since addChangeCallback isn't available)
        correctValueRange()

        // Get input states
        val isBackPressed = mc.gameSettings.keyBindBack.isKeyDown
        val playerPitch = mc.thePlayer.rotationPitch
        val isSneakPressed = Keyboard.isKeyDown(mc.gameSettings.keyBindSneak.keyCode)

        // Check activation conditions
        if ((onlySPressed.get() && !isBackPressed) ||
            (pitchCheck.get() && playerPitch < pitch.get()) ||
            (onlySneak.get() && !isSneakPressed)
        ) {
            setSneak(isSneakPressed)
            return
        }

        // Render block count if enabled
        if (showBlockCount.get()) {
            val heldItem = mc.thePlayer.heldItem
            val blockCount = heldItem?.takeIf { it.item is ItemBlock }?.stackSize ?: 0
            val fontRenderer: FontRenderer = mc.fontRendererObj
            // Use direct font rendering instead of RenderUtils
            fontRenderer.drawStringWithShadow(blockCount.toString(), 5f, 5f, 0xFFFFFF)
        }

        // Core scaffold logic
        val currentTime = System.currentTimeMillis()
        val shouldSneak = (isOverAir() || isOnEdge()) 
            && !(!MovementUtils.isMoving() && isJumpDown())

        if (shouldSneak) {
            setSneak(true)
            lastSneakTime = currentTime
        } else if (lastSneakTime != -1L) {
            val delay = Random.nextInt(minDelay.get(), maxDelay.get() + 1)
            if (currentTime - lastSneakTime > delay) {
                setSneak(false)
                lastSneakTime = -1
            }
        }
    }

    // Control sneak state
    private fun setSneak(sneak: Boolean) {
        KeyBinding.setKeyBindState(mc.gameSettings.keyBindSneak.keyCode, sneak)
    }

    // Check if player is over air
    private fun isOverAir(): Boolean {
        val feetPos = BlockPos(mc.thePlayer.posX, mc.thePlayer.posY - 0.5, mc.thePlayer.posZ)
        return BlockUtils.getBlock(feetPos) is BlockAir
    }

    // Check if player is on edge
    private fun isOnEdge(): Boolean {
        val playerPos = mc.thePlayer.position
        return listOf(
            playerPos.add(0.3, -1.0, 0.0),
            playerPos.add(-0.3, -1.0, 0.0),
            playerPos.add(0.0, -1.0, 0.3),
            playerPos.add(0.0, -1.0, -0.3)
        ).any { BlockUtils.getBlock(BlockPos(it)) is BlockAir }
    }

    // Check if jump key is pressed
    private fun isJumpDown() = mc.gameSettings.keyBindJump.isKeyDown
}