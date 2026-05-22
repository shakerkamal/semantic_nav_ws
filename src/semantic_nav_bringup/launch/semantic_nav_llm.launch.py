from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    semantic_db_path = LaunchConfiguration("semantic_db_path")
    grammar_path = LaunchConfiguration("grammar_path")
    llama_action = LaunchConfiguration("llama_action")
    parse_service = LaunchConfiguration("parse_service")

    min_confidence_percent = LaunchConfiguration("min_confidence_percent")
    max_tokens = LaunchConfiguration("max_tokens")
    target_min_len = LaunchConfiguration("target_min_len")
    target_max_len = LaunchConfiguration("target_max_len")

    temperature = LaunchConfiguration("temperature")
    top_k = LaunchConfiguration("top_k")
    top_p = LaunchConfiguration("top_p")

    llama_wait_timeout_sec = LaunchConfiguration("llama_wait_timeout_sec")
    llm_send_goal_timeout_sec = LaunchConfiguration("llm_send_goal_timeout_sec")
    llm_result_timeout_sec = LaunchConfiguration("llm_result_timeout_sec")

    debug_prompt = LaunchConfiguration("debug_prompt")
    debug_grammar = LaunchConfiguration("debug_grammar")
    allow_json_extraction_fallback = LaunchConfiguration("allow_json_extraction_fallback")

    default_semantic_db_path = PathJoinSubstitution([
        FindPackageShare("semantic_nav_semantics"),
        "config",
        "semantic_db.json",
    ])

    default_grammar_path = PathJoinSubstitution([
        FindPackageShare("semantic_nav_llm"),
        "config",
        "semantic_intent.gbnf",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "semantic_db_path",
            default_value=default_semantic_db_path,
            description="Path to semantic_nav_semantics semantic_db.json.",
        ),
        DeclareLaunchArgument(
            "grammar_path",
            default_value=default_grammar_path,
            description="Path to semantic_nav_llm GBNF grammar file.",
        ),
        DeclareLaunchArgument(
            "llama_action",
            default_value="/llama/generate_response",
            description="llama_ros GenerateResponse action endpoint.",
        ),
        DeclareLaunchArgument(
            "parse_service",
            default_value="/parse_semantic_command",
            description="Service exposed by semantic_nav_llm navigator_node.",
        ),

        DeclareLaunchArgument(
            "min_confidence_percent",
            default_value="60",
            description="Minimum confidence required for navigate intent.",
        ),
        DeclareLaunchArgument(
            "max_tokens",
            default_value="64",
            description="Request-level generation cap for intent JSON.",
        ),
        DeclareLaunchArgument(
            "target_min_len",
            default_value="1",
            description="Minimum accepted target string length.",
        ),
        DeclareLaunchArgument(
            "target_max_len",
            default_value="64",
            description="Maximum accepted target string length.",
        ),

        DeclareLaunchArgument(
            "temperature",
            default_value="0.0",
            description="LLM sampling temperature.",
        ),
        DeclareLaunchArgument(
            "top_k",
            default_value="1",
            description="LLM top-k sampling.",
        ),
        DeclareLaunchArgument(
            "top_p",
            default_value="1.0",
            description="LLM top-p sampling.",
        ),

        DeclareLaunchArgument(
            "llama_wait_timeout_sec",
            default_value="30.0",
            description="Timeout waiting for llama action server.",
        ),
        DeclareLaunchArgument(
            "llm_send_goal_timeout_sec",
            default_value="10.0",
            description="Timeout for sending llama action goal.",
        ),
        DeclareLaunchArgument(
            "llm_result_timeout_sec",
            default_value="60.0",
            description="Timeout waiting for llama result.",
        ),

        DeclareLaunchArgument(
            "debug_prompt",
            default_value="false",
            description="Print prompt sent to llama_ros.",
        ),
        DeclareLaunchArgument(
            "debug_grammar",
            default_value="false",
            description="Print GBNF grammar sent to llama_ros.",
        ),
        DeclareLaunchArgument(
            "allow_json_extraction_fallback",
            default_value="false",
            description="Allow non-strict JSON extraction fallback. Keep false normally.",
        ),

        Node(
            package="semantic_nav_llm",
            executable="navigator_node",
            name="navigator_node",
            output="screen",
            parameters=[{
                "service_name": parse_service,
                "llama_action": llama_action,
                "semantic_db_path": semantic_db_path,
                "grammar_path": grammar_path,

                "min_confidence_percent": min_confidence_percent,
                "max_tokens": max_tokens,
                "target_min_len": target_min_len,
                "target_max_len": target_max_len,

                "temperature": temperature,
                "top_k": top_k,
                "top_p": top_p,

                "llama_wait_timeout_sec": llama_wait_timeout_sec,
                "llm_send_goal_timeout_sec": llm_send_goal_timeout_sec,
                "llm_result_timeout_sec": llm_result_timeout_sec,

                "debug_prompt": debug_prompt,
                "debug_grammar": debug_grammar,
                "allow_json_extraction_fallback": allow_json_extraction_fallback,
            }],
        ),
    ])