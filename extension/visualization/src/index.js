import './assets/main.scss'
import { visualizeComponent } from './component';

const run = async () => {
    const root = document.getElementById('root');

    const data = {
        "current": {
            "LineDetector": {
                "state": "line_found",
                "variables": {
                    "current_correction": "-50.5263"
                }
            },
            "EmergencyStop": {
                "state": "in_service",
                "variables": {

                }
            }
        },
        "activated_messages": [
            "move_forward"
        ],
        "activated_services": [

        ],
        "activated_actions": [

        ],
        "received_response_messages": [
            "correction",
            "correction"
        ],
        "received_response_services": [

        ],
        "received_response_actions": [

        ],
        "received_feedback_actions": [

        ],
        "definition": { "name": "LineFollowerController", "components": [{ "name": "LineDetector", "messages": ["correction", "no_line"], "services": [], "actions": [], "behaviour": [{ "name": "awaiting", "initial": true, "transitions": [{ "next": "no_line", "type": " response" }, { "next": "line_found", "type": " response" }] }, { "name": "line_found", "initial": false, "transitions": [{ "next": "no_line", "type": " response" }, { "next": null, "type": " response", "assignment": " current_correction := value" }] }, { "name": "no_line", "initial": false, "transitions": [{ "next": "line_found", "type": " response" }] }] }, { "name": "EmergencyStop", "messages": ["stop", "continue"], "services": [], "actions": [], "behaviour": [{ "name": "in_service", "initial": true, "transitions": [{ "next": "stopped", "type": " response" }] }, { "name": "stopped", "initial": false, "transitions": [{ "next": "in_service", "type": " response" }] }] }, { "name": "TurtlebotPlatform", "messages": ["move_forward", "halt"], "services": [], "actions": [] }] }
    }

    const definition = data.definition;

    const title = document.createElement('h1');
    title.innerHTML = definition.name;

    const grid = document.createElement('div');
    grid.classList.add('grid');

    root.appendChild(title);
    root.appendChild(grid);

    for (const component of definition.components) {
        grid.appendChild(visualizeComponent(component, data.current[component.name]));
    }
};

run();