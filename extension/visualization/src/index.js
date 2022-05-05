import { fsl_to_svg_string } from 'jssm-viz';
import './assets/main.scss'

const run = async () => {
    const root = document.getElementById('root');
    const stateMachineCode = `
        in_service => stopped => in_service;

        flow: left;

        state in_service : { corners: rounded; };
        state stopped : { corners: rounded; };
    `;
    
    const svg = await fsl_to_svg_string(stateMachineCode);
    
    root.innerHTML = svg;
};

run();