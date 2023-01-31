import { ComponentType } from "./component-type";

export interface VisibleComponent {
  active: boolean;
  componentType: ComponentType;
  reset: any;
}
