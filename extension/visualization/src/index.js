import './assets/main.scss'
import { updateState, visualizeComponent } from './component';

let lastDefinition = null;
let lastSerialized = null;

const handleData = (data) => {
    const current = JSON.stringify(data.definition);

    if (lastSerialized !== current) {
        const root = document.getElementById('root');

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

        lastSerialized = current;
        lastDefinition = definition;
    }

    setTimeout(() => updateState(lastDefinition, data.current));
};

const data = {
    "current": {
        "LineDetector": {
            "state": "awaiting",
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
    "definition": {"name":"LineFollowerController","components":[{"name":"LineDetector","messages":["correction","no_line"],"services":[],"actions":[],"behaviour":{"variables":["current_correction"],"states":[{"name":"awaiting","initial":true,"transitions":[{"next":"no_line","type":"response","communication":"no_line"},{"next":"line_found","type":"response","communication":"correction"}]},{"name":"line_found","initial":false,"transitions":[{"next":"no_line","type":"response","communication":"no_line"},{"next":null,"type":"response","communication":"correction","assignment":"current_correction := value"}]},{"name":"no_line","initial":false,"transitions":[{"next":"line_found","type":"response","communication":"correction"}]}]}},{"name":"EmergencyStop","messages":["stop","continue"],"services":[],"actions":[],"behaviour":{"variables":[],"states":[{"name":"in_service","initial":true,"transitions":[{"next":"stopped","type":"response","communication":"stop"}]},{"name":"stopped","initial":false,"transitions":[{"next":"in_service","type":"response","communication":"continue"}]}]}},{"name":"TurtlebotPlatform","messages":["move_forward","halt"],"services":[],"actions":[]}]}
}

handleData(data);

const clone = { ...data };

setTimeout(() => step1(), 2000);
const step1 = () => {
    clone.current.LineDetector.state = 'line_found';
    handleData(clone);

    setTimeout(() => step2(), 4000);
}

const step2 = () => {
    clone.current.EmergencyStop.state = 'stopped';
    handleData(clone);
setTimeout(() => step3(), 4000);
}

const step3 = () => {
    clone.current.LineDetector.state = 'no_line';
    handleData(clone);
setTimeout(() => step4(), 4000);
}

const step4 = () => {
    clone.current.LineDetector.state = 'awaiting';
    clone.current.EmergencyStop.state = 'in_service';
    handleData(clone);
setTimeout(() => step1(), 4000);
}