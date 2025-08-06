// ------------------------------------------------------------------------------------------------------------------------ //
// ----------------------------------------------------- LIBRARIES -------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------ //

// -------------------- Unity -------------------- //
using UnityEngine.EventSystems;
using UnityEngine;

public class gripper_button_check : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    // -------------------- Int -------------------- //
    public bool isGripButton = true; // true为夹持按钮，false为放开按钮
    private ur_data_processing urDataProcessor;

    void Start()
    {
        // 获取ur_data_processing实例
        urDataProcessor = FindObjectOfType<ur_data_processing>();
        if (urDataProcessor == null)
        {
            Debug.LogError("无法找到ur_data_processing实例！");
        }
    }

    // -------------------- 按钮按下 -------------------- //
    public void OnPointerDown(PointerEventData eventData)
    {
        if (urDataProcessor != null)
        {
            if (isGripButton)
            {
                // 夹持操作
                urDataProcessor.SendGripCommand();
            }
            else
            {
                // 放开操作
                urDataProcessor.SendReleaseCommand();
            }
        }
    }

    // -------------------- 按钮释放 -------------------- //
    public void OnPointerUp(PointerEventData eventData)
    {
        // 对于夹持/放开操作，我们不需要在松开按钮时做任何事
        // 可以在此添加特定的松开反馈逻辑，如有需要
    }
} 