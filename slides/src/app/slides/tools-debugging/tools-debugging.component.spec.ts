import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ToolsDebuggingComponent } from './tools-debugging.component';

describe('LanguageDebuggingComponent', () => {
  let component: ToolsDebuggingComponent;
  let fixture: ComponentFixture<ToolsDebuggingComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ToolsDebuggingComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ToolsDebuggingComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
