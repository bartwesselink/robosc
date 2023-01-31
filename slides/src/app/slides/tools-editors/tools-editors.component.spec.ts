import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ToolsEditorsComponent } from './tools-editors.component';

describe('OverviewArchitectureComponent', () => {
  let component: ToolsEditorsComponent;
  let fixture: ComponentFixture<ToolsEditorsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ToolsEditorsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ToolsEditorsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
