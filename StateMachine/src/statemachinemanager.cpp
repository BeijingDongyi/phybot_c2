/* include */
#include "StateMachine/include/statemachinemanager.h"


void StateMachineManager::GetDataFromPackage(DataPackage& DataPackage)
{
    NextState = DataPackage.NextState;
}


void StateMachineManager::SetDataToPackage(DataPackage& DataPackage)
{
    DataPackage.CurrentState = CurrentState;
}

void StateMachineManager::run(DataPackage& data)
{
    if (CurrentState != NextState) {

        /*
         * 保护机制：检查目标状态的模型是否已加载完毕
         * 若未加载完成则阻塞切换，继续运行当前状态
         * 防止切换到未就绪的状态导致崩溃
         */
        auto nextHandler = handlers.find(NextState);
        /* IDLE 无策略输出、无需加载模型，直接放行（IDLE 不注册 handler）
         * 注：不能用 handlers[NextState] 查，IDLE 会使 map 插入 null 破坏后续 find */
        const bool target_is_idle = (NextState == State::IDLE);
        bool targetReady = target_is_idle ||
                           (nextHandler != handlers.end() && nextHandler->second->isLoaded());

        if (!targetReady) {
            /* 目标状态模型未就绪，阻塞切换 */
            std::cout << "[Blocked] Target state " << static_cast<int>(NextState)
                      << " 模型未加载完毕，继续保持原状态"
                      << static_cast<int>(CurrentState) << std::endl;
        } else {
            /* 执行状态切换 */
            handleEvent(CurrentState, Event::EXIT, data);

            std::cout << "Success transition: " << static_cast<int>(CurrentState)
            << " → " << static_cast<int>(NextState) << std::endl;
            /* 更新当前状态 */
            CurrentState = NextState;
        }
    }

    /* 进入零位时触发后台线程加载其余模型（测试可关闭） */
    if (bg_loading_enabled_ && CurrentState == State::ZERO) {
        startBackgroundLoading();
    }

    /* 执行当前状态的 RUN 逻辑 */
    handleEvent(CurrentState, Event::RUN, data);
}
