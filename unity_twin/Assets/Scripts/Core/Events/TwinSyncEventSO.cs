using RosMessageTypes.Miracle;
using UnityEngine;

namespace MiracleTwin.Core
{
    [CreateAssetMenu(menuName = "MIRACLE/Events/Twin Sync")]
    public class TwinSyncEventSO : GameEventSO<TwinSyncStatusMsg> { }
}
