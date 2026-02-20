using UnityEngine.UIElements;

namespace MiracleTwin.UI
{
    public static class LabelExtensions
    {
        public static void SetText(this Label label, string text)
        {
            label.text = text;
        }
    }
}
